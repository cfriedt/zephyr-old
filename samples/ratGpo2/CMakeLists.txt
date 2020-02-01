# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(rfPacketErrorRate)

zephyr_include_directories(
	src
)

target_sources(app PRIVATE
	src/main.c
	src/power.c
	src/smartrf_settings.c
	src/ti/drivers/pin/PINCC26XX.c
)
