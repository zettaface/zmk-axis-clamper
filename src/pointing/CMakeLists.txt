# Copyright (c) 2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources_ifdef(CONFIG_ZMK_AXIS_CLAMPER app PRIVATE axis_clamper.c)
