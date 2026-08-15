# Notes: Building, Signing, and Installing XCSoar on a Personal iPhone

This documents what was done to get a locally-built XCSoar onto a personal
(non-paid-Developer-Program) iPhone using the free "Apple Development" signing
identity.

## 1. Build failure: `make` couldn't find the `.rsp` file

```
clang++: error: no such file or directory: '@output/IOS64/bin/vali-xcs.rsp'
```

Cause: macOS ships `/usr/bin/make` = GNU Make **3.81** (Apple keeps it old for
licensing reasons). [build/link.mk](../build/link.mk) uses the `$(file >...)`
function to write linker response files, which requires GNU Make **4.0+**.
With 3.81 the `.rsp` file is silently never created, so the link step fails.

Fix: use Homebrew's `gmake` (4.4.1) instead of the system `make`, e.g.:

```sh
gmake TARGET=IOS64 -j12
```

To make plain `make` resolve to the newer version permanently, add this to
`~/.zshrc` (Homebrew's `make` formula ships a `gnubin` shim dir):

```sh
export PATH="/opt/homebrew/opt/make/libexec/gnubin:$PATH"
```

## 2. Building an IPA with a bundle ID matching an existing profile

By default `build/ios.mk` sets `IOS_APP_BUNDLE_IDENTIFIER` to `XCSoar`
(or `XCSoar-testing` when `TESTING=y`), **not** `org.xcsoar.XCSoar`.

A free 7-day development provisioning profile was already present at:

```
~/Library/Developer/Xcode/UserData/Provisioning Profiles/c2450d9d-e58c-416e-937c-9461b8d47aac.mobileprovision
```

It was created for app ID `368S9Y67U6.org.xcsoar.XCSoar` and already contained
this iPhone's device UDID (`00008150-001E60263A03401C`), so it was reused
instead of creating a new one.

Because the profile's app ID is `org.xcsoar.XCSoar`, the build had to override
the bundle identifier to match:

```sh
rm -f output/IOS64/Info.plist output/IOS64/Info.plist.xml output/IOS64/xcsoar.ipa
make TARGET=IOS64 IOS_APP_BUNDLE_IDENTIFIER=org.xcsoar.XCSoar ipa
```

The explicit `rm` is required because Make only tracks file timestamps, not
variable changes — without deleting the old `Info.plist`/`Info.plist.xml`/`.ipa`
first, Make thinks they're already up to date and won't regenerate them with
the new bundle identifier.

## 3. Signing

The signing identity available in the keychain was a free "Apple Development"
certificate (not "Apple Distribution", which requires a paid account):

```
security find-identity -v -p codesigning
# "Apple Development: george.downing2@icloud.com (7SS5RW8MV3)"
```

Its team (`368S9Y67U6`, from the cert's `OU` field) matches the provisioning
profile's `TeamIdentifier`, so it was used with [darwin/sign.sh](sign.sh):

```sh
export IOS_PROFILE_PATH="$HOME/Library/Developer/Xcode/UserData/Provisioning Profiles/c2450d9d-e58c-416e-937c-9461b8d47aac.mobileprovision"
export APPLE_DISTRIBUTION_CERTIFICATE_NAME="Apple Development: george.downing2@icloud.com (7SS5RW8MV3)"
darwin/sign.sh
```

This produces `output/IOS64/xcsoar-signed.ipa`.

### First signing attempt failed: "invalid entitlements"

The very first sign+install attempt used the IPA that still had the default
`CFBundleIdentifier` of `XCSoar`, while the profile's entitlements declared
`application-identifier = 368S9Y67U6.org.xcsoar.XCSoar`. That mismatch made
`installd` reject the app with:

```
The executable was signed with invalid entitlements.
```

Rebuilding the IPA with `IOS_APP_BUNDLE_IDENTIFIER=org.xcsoar.XCSoar` (step 2
above) and re-signing fixed this.

## 4. Installing

The device was found via:

```sh
xcrun devicectl list devices
```

There were two entries named "George's iphone" (one connected, one
unavailable/stale), so the unique **device identifier** was used instead of
the name to avoid ambiguity:

```sh
export IOS_DEVICE_NAME="21134652-DBE4-5C81-87AC-0D16199F8119"
export IOS_BUNDLE_ID="org.xcsoar.XCSoar"
darwin/install.sh
```

This installs via `xcrun devicectl device install app` and then tries to
launch the app with console output attached.

### Launch failed the first time: certificate not trusted

```
Unable to launch org.xcsoar.XCSoar because it has an invalid code signature,
inadequate entitlements or its profile has not been explicitly trusted by the user.
```

This is expected for a fresh personal-team install — iOS blocks apps signed
by an untrusted developer certificate until the user explicitly trusts it:

**Settings → General → VPN & Device Management → "Apple Development:
george.downing2@icloud.com" → Trust.**

After trusting, the app launches normally from the home screen or by
re-running `darwin/install.sh`.

## Caveats / expiry

- The free development provisioning profile expires **2026-08-22** (7 days
  from creation). After that, a new profile must be generated (easiest via
  Xcode's automatic signing) and the sign/install steps repeated.
- `darwin/.env.example` is written for the **App Store Connect / paid
  Distribution** signing flow (`IOS_CERTIFICATE_NAME`,
  `MACOS_CERTIFICATE_NAME`, etc.). For this local personal-device install we
  used the "Apple Development" identity name directly for
  `APPLE_DISTRIBUTION_CERTIFICATE_NAME` instead — the variable name is just a
  label consumed by [sign.sh](sign.sh), it works with any valid codesigning
  identity string.
