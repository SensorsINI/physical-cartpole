#ifndef RPGD_ON_TARGET_TEST_H
#define RPGD_ON_TARGET_TEST_H

/*
 * Motor-safe Zynq PS RPGD timing/parity harness.
 *
 * Default firmware leaves this compiled out. To build the test ELF without
 * changing boot behavior of the Debug app:
 *   Firmware/Scripts/build_rpgd_on_target_elf.sh
 * UART is 230400 8N1. Do not flash from that script while the board is in use.
 *
 * To opt in from the regular Vitis Debug build, uncomment RPGD_ON_TARGET_TEST
 * in hardware_bridge.h and rebuild. That path never installs CONTROL_Loop.
 */
void rpgd_on_target_test_run(void);

#endif
