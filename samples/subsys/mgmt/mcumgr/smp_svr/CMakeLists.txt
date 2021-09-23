cmake_minimum_required(VERSION 3.13.1)
# Top-level CMakeLists.txt for the skeleton application.
#
# Copyright (c) 2017 Open Source Foundries Limited
#
# SPDX-License-Identifier: Apache-2.0
#
# This provides a basic application structure suitable for communication using
# mcumgr.  It can be used as a starting point for new applications.

# Standard Zephyr application boilerplate.
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smp_svr)

set(CMAKE_VERBOSE_MAKEFILE ON)

zephyr_compile_definitions(CBOR_NO_FLOATING_POINT=1 CBOR_NO_HALF_FLOAT_TYPE=1)

target_sources(app PRIVATE src/main.c)
target_sources_ifdef(CONFIG_MCUMGR_SMP_BT app PRIVATE src/bluetooth.c)
target_sources_ifdef(CONFIG_MCUMGR_SMP_UDP app PRIVATE src/udp.c)
