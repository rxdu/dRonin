GDB = arm-none-eabi-gdb

## J-Link
# In order to use JLink to load the code to Flash, you need the following 2 steps:
# 1. start a gdb server first: make jlink_gdb_server
# 2. load code and debug/run: make jlink_gdb_debug
jlink_gdb_server:
	JLinkGDBServer -if SWD -device STM32F427VI

jlink_gdb_debug:
	$(GDB) -x ./scripts/gdb/jlink-debug.gdbinit build/fw_pixracer/fw_pixracer.elf

jlink_gdb_session:
	JLinkGDBServer -if SWD -device STM32F427VI &
	$(GDB) -x ./scripts/gdb/jlink-debug.gdbinit build/fw_pixracer/fw_pixracer.elf

jlink_flash_pixracer:
	JLinkExe -if SWD -device STM32F427VI -speed 4000 -CommanderScript ./scripts/jlink/jlink-flash-pixracer.jlink