--------------------------------------------------------------------------------
-- Package: ide64_ata_pkg
--
-- Description: CPU-side register map of the IDE64 ATA device (ide64_ata).
--              Byte offsets within its I/O window. Offsets $200-$3FF are the
--              512-byte sector buffer (byte n = byte n of the sector; the C64
--              sees the word at 2k as low byte 2k, high byte 2k+1).
--              Keep in sync with software/io/c64/ide64.h.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package ide64_ata_pkg is

    -- Task file, as last written by the C64 (or by the CPU as the command result)
    constant c_ide_features : unsigned(3 downto 0) := X"0"; -- R
    constant c_ide_error    : unsigned(3 downto 0) := X"1"; -- R/W
    constant c_ide_nsect    : unsigned(3 downto 0) := X"2"; -- R/W
    constant c_ide_sector   : unsigned(3 downto 0) := X"3"; -- R/W
    constant c_ide_lcyl     : unsigned(3 downto 0) := X"4"; -- R/W
    constant c_ide_hcyl     : unsigned(3 downto 0) := X"5"; -- R/W
    constant c_ide_select   : unsigned(3 downto 0) := X"6"; -- R/W device/head
    constant c_ide_command  : unsigned(3 downto 0) := X"7"; -- R

    -- R: status as the C64 sees it. W: status bits other than BSY (7) and DRQ (3),
    -- which the hardware owns.
    constant c_ide_status   : unsigned(3 downto 0) := X"8";

    -- W: completes the current step: clears BSY and sets up the data phase.
    --    bit 0: arm a data phase (word pointer = 0, DRQ on)
    --    bit 1: direction: 1 = C64 writes the buffer (data-out), 0 = C64 reads it
    --    bit 2: data-in only: set BSY again when the C64 has read the whole buffer
    --           (more sectors follow). Data-out always sets BSY when the buffer is full.
    -- R: bit 0 = data phase active, bit 1 = direction, bit 2 = busy-at-end, bit 7 = BSY
    constant c_ide_control  : unsigned(3 downto 0) := X"9";

    -- R: pending events. W: write 1 to clear.
    -- While c_ide_ev_reset is pending, CPU writes to the task file, status and
    -- control are ignored, so a late completion can't overwrite the reset state.
    constant c_ide_events   : unsigned(3 downto 0) := X"A";
    constant c_ide_ev_cmd   : natural := 0; -- the C64 wrote a command (BSY is set)
    constant c_ide_ev_buf   : natural := 1; -- buffer drained (data-in, busy-at-end) or filled (data-out); BSY is set
    constant c_ide_ev_reset : natural := 2; -- SRST (device control bit 2, 1 -> 0) or C64 reset

    -- R/W: bits 3..0: device present (bit = interface * 2 + device). bit 7: IRQ enable.
    constant c_ide_present  : unsigned(3 downto 0) := X"B";
    constant c_ide_devctl   : unsigned(3 downto 0) := X"C"; -- R: device control (reg 14)
    constant c_ide_iface    : unsigned(3 downto 0) := X"D"; -- R: bit 0 = interface, bit 7 = IDE64 cart active
    constant c_ide_pointer  : unsigned(3 downto 0) := X"E"; -- R: word pointer (debug)

end package;
