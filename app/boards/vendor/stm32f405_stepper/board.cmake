# SPDX-License-Identifier: Apache-2.0

board_runner_args(dfu-util "--pid=0483:df11" "--alt=0" "--dfuse")
board_runner_args(jlink "--device=STM32F405RG" "--speed=4000")
board_runner_args(pyocd "--target=stm32f405rg")
board_runner_args(openocd --config interface/cmsis-dap.cfg -c "adapter speed 500" --config target/stm32f4x.cfg)
include(${ZEPHYR_BASE}/boards/common/dfu-util.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)

set(BOARD_FLASH_RUNNER openocd)