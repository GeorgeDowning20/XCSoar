// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "OGNClient.hpp"

#include "event/Loop.hxx"
#include "event/Call.hxx"
#include "event/net/cares/Channel.hxx"
#include "net/SocketAddress.hxx"
#include "net/IPv4Address.hxx"
#include "net/IPv6Address.hxx"
#include "util/BindMethod.hxx"
#include "util/SpanCast.hxx"
#include "util/PrintException.hxx"
#include "util/Exception.hxx"
#include "util/EnvParser.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <iostream>

namespace {
constexpr auto CONNECT_TIMEOUT = std::chrono::seconds(15);
constexpr auto RECONNECT_DELAY = std::chrono::seconds(25);
constexpr auto RECEIVE_TIMEOUT = std::chrono::minutes(2);
constexpr unsigned APRS_FILTER_PORT = 14580;
constexpr unsigned APRS_ALTERNATE_PORT = 14501;
constexpr std::size_t RX_BUFFER_CAPACITY = 16384;
} // namespace

/**
 * Synthesize an IPv6 candidate for an IPv4 address using the RFC 6052
 * well-known NAT64 prefix (64:ff9b::/96).  On IPv6-only/NAT64 cellular
 * networks (common on iOS carriers), literal IPv4 addresses have no
 * route at all, so a plain connect() fails instantly with "No route to
 * host" regardless of which port is tried; this gives such networks a
 * usable path when the carrier's NAT64 gateway uses the standard
 * prefix instead of a custom one.
 */
static AllocatedSocketAddress
SynthesizeNat64(const IPv4Address &v4) noexcept
{
  const uint32_t a = v4.GetNumericAddress();
  return AllocatedSocketAddress(IPv6Address(0x0064, 0xff9b, 0, 0, 0, 0,
                                            uint16_t(a >> 16), uint16_t(a),
                                            v4.GetPort()));
}

OGNClient::OGNClient(EventLoop &_loop, Cares::Channel &_cares,
                     OGNAprsHandler &_handler,
                     std::string &&_host, unsigned _port,
                     std::string &&_user, std::string &&_pass,
                     std::string &&_filter) noexcept
  : ConnectSocketHandler(),
    loop(_loop),
    cares(_cares),
    handler(_handler),
    host(std::move(_host)),
    port(_port),
    user(std::move(_user)),
    pass(std::move(_pass)),
    filter(std::move(_filter)),
    resolver_handler(*this),
    connector(loop, *this),
    read_event(loop, BIND_THIS_METHOD(OnReadReady)),
    reconnect_timer(loop, BIND_THIS_METHOD(OnReconnectTimer)),
    receive_timeout(loop, BIND_THIS_METHOD(OnReceiveTimeout)) {}

OGNClient::~OGNClient() noexcept
{
  BlockingCall(loop, [this](){ InternalStop(); });
}

void
OGNClient::SetFilter(std::string &&_filter) noexcept
{
  BlockingCall(loop, [this, f = std::move(_filter)]() mutable {
    InternalSetFilter(std::move(f));
  });
}

void
OGNClient::InternalSetFilter(std::string &&_filter) noexcept
{
  if (filter == _filter)
    return;

  filter = std::move(_filter);

  if (!IsConnected())
    return;

  char line[512];
  const int n = std::snprintf(line, sizeof(line), "#filter %s\r\n",
                              filter.c_str());
  if (n <= 0 || unsigned(n) >= sizeof(line))
    return;

  (void)read_event.GetSocket().WriteNoWait(
    AsBytes(std::string_view(line, unsigned(n))));
}

void
OGNClient::Start() noexcept
{
  BlockingCall(loop, [this](){ InternalStart(); });
}

void
OGNClient::InternalStart() noexcept
{
  std::cerr << "OGN\tlookup\t" << host << ':' << port << std::endl;
  BeginLookup();
}

void
OGNClient::Stop() noexcept
{
  BlockingCall(loop, [this](){ InternalStop(); });
}

void
OGNClient::InternalStop() noexcept
{
  reconnect_timer.Cancel();
  receive_timeout.Cancel();
  resolver_job.reset();
  CloseConnection();
}

void
OGNClient::BeginLookup() noexcept
{
  if (resolver_job.has_value())
    return;

  /* c-ares only reads the system nameserver config once at startup;
     reload it before each (re)connect attempt so switching between
     WiFi and cellular doesn't leave us querying a now-unreachable
     stale DNS server ("No route to host" on every address). */
  cares.Reinit();

  resolver_job.emplace(resolver_handler, port);
  resolver_job->Start(cares, host.c_str());
}

void
OGNClient::TryConnect(std::forward_list<AllocatedSocketAddress> addresses) noexcept
{
  /* collect the native IPv4 candidates before appending NAT64-
     synthesized IPv6 ones after them, so normal networks are
     unaffected and try order for the real addresses is unchanged */
  std::forward_list<IPv4Address> native_v4;
  for (const auto &a : addresses)
    if (a.GetFamily() == AF_INET)
      native_v4.emplace_front(a);

  if (!native_v4.empty()) {
    auto tail = addresses.before_begin();
    for (auto it = addresses.begin(); it != addresses.end(); ++it)
      tail = it;

    for (const auto &v4 : native_v4)
      tail = addresses.emplace_after(tail, SynthesizeNat64(v4));
  }

  pending_addresses = std::move(addresses);
  TryNextAddress();
}

void
OGNClient::TryNextAddress() noexcept
{
  if (trying_next_address) {
    /* ConnectSocket::Connect() calls OnSocketConnectError()
       synchronously for an immediate failure (e.g. "No route to
       host"), which re-enters this function while the outer call's
       own loop below is still on the stack; let that outer loop carry
       on with the next address itself instead of running the
       exhaustion handling (port-fallback/reconnect) once per
       recursion level. */
    return;
  }

  trying_next_address = true;

  while (!pending_addresses.empty()) {
    AllocatedSocketAddress a = std::move(pending_addresses.front());
    pending_addresses.pop_front();

    if (connector.IsPending())
      connector.Cancel();

    if (connector.Connect(a, CONNECT_TIMEOUT)) {
      trying_next_address = false;
      return;
    }
  }

  trying_next_address = false;

  if (port == APRS_FILTER_PORT) {
    /* Some mobile networks block the standard filtered APRS-IS port.
       Retry the same OGN service on its alternate plain TCP port right
       away, without waiting for the normal reconnect delay. */
    TogglePort();
    std::cerr << "OGN\tport-fallback\t" << host << ':' << port << std::endl;
    BeginLookup();
    return;
  }

  ScheduleReconnect();
}

void
OGNClient::OnSocketConnectSuccess(UniqueSocketDescriptor fd) noexcept
{
  if (read_event.IsDefined()) {
    /* defensive: ignore a stray duplicate connect-success (should not
       happen now that the reconnect timer is cancelled below, but
       don't corrupt the existing connection if it does) */
    std::cerr << "OGN\tduplicate-connect\t" << host << ':' << port << std::endl;
    return;
  }

  std::cerr << "OGN\tconnected\t" << host << ':' << port << std::endl;

  /* a reconnect may have been scheduled by an earlier failed attempt
     while this one was still resolving/connecting; cancel it now so
     it doesn't fire a redundant reconnect while we're happily
     connected (#the actual cause of the "connected" twice / crash). */
  reconnect_timer.Cancel();
  pending_addresses.clear();

  read_event.Open(fd.Release());
  read_event.ScheduleRead();
  ScheduleReceiveTimeout();
  rx_buffer.clear();
  try {
    rx_buffer.reserve(RX_BUFFER_CAPACITY);
  } catch (...) {
    std::cerr << "OGN\talloc-error\t" << host << ':' << port << std::endl;
    ScheduleReconnect();
    return;
  }
  SendLogin();
}

void
OGNClient::OnSocketConnectError(std::exception_ptr error) noexcept
{
  std::cerr << "OGN\tconnect-error\t" << host << ':' << port << '\t'
            << GetFullMessage(error) << std::endl;

  /* fall through to the next resolved address (e.g. an unreachable
     IPv6 route) instead of giving up for a full reconnect cycle */
  TryNextAddress();
}

void
OGNClient::SendLogin() noexcept
{
  char line[512];
  const int n =
    std::snprintf(line, sizeof(line),
                  "user %s pass %s vers XCSoar 0.1 filter %s\r\n",
                  user.c_str(), pass.c_str(), filter.c_str());
  if (n <= 0 || unsigned(n) >= sizeof(line))
    return;

  (void)read_event.GetSocket().WriteNoWait(
    AsBytes(std::string_view(line, unsigned(n))));
}

void
OGNClient::CloseConnection() noexcept
{
  receive_timeout.Cancel();
  read_event.Close();
  pending_addresses.clear();

  if (connector.IsPending())
    connector.Cancel();
}

void
OGNClient::ScheduleReceiveTimeout() noexcept
{
  receive_timeout.Schedule(RECEIVE_TIMEOUT);
}

void
OGNClient::TogglePort() noexcept
{
  port = port == APRS_FILTER_PORT ? APRS_ALTERNATE_PORT : APRS_FILTER_PORT;
}

void
OGNClient::ScheduleReconnect() noexcept
{
  /* alternate ports on every reconnect cycle instead of latching onto
     whichever one TryNextAddress last fell back to: a network that
     blocks one APRS-IS port (or resets the connection right after
     accepting it) may still allow the other one. */
  TogglePort();

  std::cerr << "OGN\treconnect\t" << host << ':' << port
            << "\tin=" << RECONNECT_DELAY.count() << "s" << std::endl;
  CloseConnection();
  reconnect_timer.Schedule(RECONNECT_DELAY);
}

void
OGNClient::OnReconnectTimer() noexcept
{
  BeginLookup();
}

void
OGNClient::OnReceiveTimeout() noexcept
{
  if (!read_event.IsDefined())
    return;

  std::cerr << "OGN\treceive-timeout\t" << host << ':' << port << std::endl;
  ScheduleReconnect();
}

void
OGNClient::OnReadReady(unsigned events) noexcept
{
  if (events & SocketEvent::HANGUP) {
    std::cerr << "OGN\thangup\t" << host << ':' << port << std::endl;
    ScheduleReconnect();
    return;
  }

  if (events & SocketEvent::ERROR) {
    std::cerr << "OGN\tsocket-error\t" << host << ':' << port << std::endl;
    ScheduleReconnect();
    return;
  }

  std::array<std::byte, 4096> buf{};
  const ssize_t nbytes =
    read_event.GetSocket().Read(std::span<std::byte>{buf.data(), buf.size()});
  if (nbytes <= 0) {
    std::cerr << "OGN\teof\t" << host << ':' << port << std::endl;
    ScheduleReconnect();
    return;
  }

  if (GetEnvBool("XCS_CLOUD_DEBUG"))
    std::cerr << "OGN\trx\t" << nbytes << " bytes" << std::endl;

  ScheduleReceiveTimeout();

  try {
    ConsumeInput({(const char *)buf.data(), (std::size_t)nbytes});
  } catch (...) {
    std::cerr << "OGN\talloc-error\t" << host << ':' << port << std::endl;
    ScheduleReconnect();
  }
}

void
OGNClient::ConsumeInput(std::string_view chunk)
{
  rx_buffer.append(chunk.data(), chunk.size());

  std::size_t cut = 0;
  while (true) {
    const auto nl = rx_buffer.find('\n', cut);
    if (nl == std::string::npos)
      break;

    std::string_view line(rx_buffer.data() + cut, nl - cut);
    while (!line.empty() && line.back() == '\r')
      line.remove_suffix(1);

    handler.OnAprsLine(line);
    cut = nl + 1;
  }

  if (cut > 0)
    rx_buffer.erase(0, cut);

  static constexpr std::size_t MAX_ACCUM = 256 * 1024;
  if (rx_buffer.size() > MAX_ACCUM)
    rx_buffer.clear();
}
