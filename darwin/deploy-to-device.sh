#!/bin/bash
set -euo pipefail

# Build, sign, and install XCSoar on the iPhone configured in darwin/.env.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# shellcheck disable=SC1091
source "$SCRIPT_DIR/.env"

# Force a clean regeneration so IOS_BUNDLE_ID actually takes effect (make only
# tracks file timestamps, not variable changes).
rm -f output/IOS64/Info.plist output/IOS64/Info.plist.xml output/IOS64/xcsoar.ipa

gmake -j12 TARGET=IOS64 IOS_APP_BUNDLE_IDENTIFIER="$IOS_BUNDLE_ID" ipa
"$SCRIPT_DIR/sign.sh"
"$SCRIPT_DIR/install.sh"
