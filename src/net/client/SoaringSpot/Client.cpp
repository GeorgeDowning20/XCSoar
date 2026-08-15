// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Client.hpp"
#include "Parser.hpp"
#include "net/http/Progress.hpp"
#include "net/http/CoDownloadToFile.hpp"
#include "lib/curl/CoStreamRequest.hxx"
#include "lib/curl/Easy.hxx"
#include "lib/curl/Setup.hxx"
#include "lib/fmt/RuntimeError.hxx"
#include "io/StringOutputStream.hxx"

namespace SoaringSpot {

static Co::Task<std::string>
CoGet(CurlGlobal &curl, const char *url, ProgressListener &progress)
{
  CurlEasy easy{url};
  Curl::Setup(easy);
  easy.SetOption(CURLOPT_ACCEPT_ENCODING, "");
  easy.SetOption(CURLOPT_FOLLOWLOCATION, 1L);
  easy.SetOption(CURLOPT_MAXREDIRS, 10L);
  easy.SetTimeout(60);

  const Net::ProgressAdapter progress_adapter{easy, progress};

  StringOutputStream sos;
  const auto response =
    co_await Curl::CoStreamRequest(curl, std::move(easy), sos);

  if (response.status != 200)
    throw FmtRuntimeError("SoaringSpot status {}", response.status);

  co_return std::move(sos).GetValue();
}

Co::Task<std::vector<Contest>>
ListContests(CurlGlobal &curl, ProgressListener &progress)
{
  const std::string html =
    co_await CoGet(curl, "https://www.soaringspot.com/en_gb/", progress);
  co_return ParseContests(html);
}

Co::Task<std::vector<File>>
ListFiles(CurlGlobal &curl, std::string contest_url,
          ProgressListener &progress)
{
  const std::string url = contest_url + "/downloads";
  const std::string html = co_await CoGet(curl, url.c_str(), progress);
  co_return ParseFiles(html);
}

Co::Task<Results>
ListResults(CurlGlobal &curl, std::string contest_url,
            ProgressListener &progress)
{
  const std::string url = contest_url + "/results";
  const std::string html = co_await CoGet(curl, url.c_str(), progress);
  co_return ParseResults(html);
}

Co::Task<TaskDetails>
DownloadTask(CurlGlobal &curl, std::string task_url,
             ProgressListener &progress)
{
  const std::string html = co_await CoGet(curl, task_url.c_str(), progress);
  co_return ParseTaskDetails(html);
}

Co::Task<bool>
DownloadFile(CurlGlobal &curl, std::string url, AllocatedPath path,
             ProgressListener &progress)
{
  co_await Net::CoDownloadToFile(curl, url.c_str(), nullptr, nullptr,
                                 path, nullptr, progress);
  co_return true;
}

} // namespace SoaringSpot
