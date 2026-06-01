#!/usr/bin/env python3

import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_ROOT = os.path.dirname(SCRIPT_DIR)
if SDK_ROOT not in sys.path:
    sys.path.insert(0, SDK_ROOT)

from el_a3_sdk.controller_profiles import main


if __name__ == "__main__":
    raise SystemExit(main())
