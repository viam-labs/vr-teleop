#!/bin/sh
# Wrapper to set library path for bundled libsurvive before launching the module binary.
DIR="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="${DIR}/libsurvive/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export DYLD_LIBRARY_PATH="${DIR}/libsurvive/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
exec "${DIR}/bin/vive" "$@"
