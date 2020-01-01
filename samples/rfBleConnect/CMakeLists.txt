# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(rfPacketErrorRate)

target_sources(app PRIVATE
	src/RFQueue.c
	src/config.c
	src/main_nortos.c
	src/menu_nortos.c
	src/power.c
	src/rfBleConnect_nortos.c
	src/rx.c
	src/smartrf_settings/smartrf_settings_ble.c
	src/smartrf_settings/smartrf_settings_predefined.c
	src/smartrf_settings/smartrf_settings.c
	src/tx.c
)
