# IDE64 cartridge emulation: findings and plan

Status: feasibility checked and the hardware spec extracted from VICE (2026-11-10).
Phase 1 implemented, simulated and built for U2+; not yet run on hardware (2026-11-12).

Reference implementation: VICE, `E:\GitHub\svn-mirror\vice`
- `src/c64/cart/ide64.c`: cart logic, IO map, banking, RTC, CRT loading
- `src/c64/cart/c64cartmem.c`: which C64 address ranges go to IDE64 ROM/RAM (search `CARTRIDGE_IDE64`)
- `src/core/ata.c`: ATA drive model (registers, commands, IDENTIFY data)
- `src/core/ds1202_1302.c`: RTC model

## 1. Findings in this repo

### Where cartridge logic lives
- All cart emulation is one `case cart_logic_d is` statement in
  `fpga/cart_slot/vhdl_source/all_carts_v5.vhd`. It is instantiated from
  `fpga/cart_slot/vhdl_source/slot_server_v4.vhd` (`i_cart_logic`, around line 757).
- Each cart type drives:
  - bus control: `game_n`, `exrom_n`, `serve_rom`, `serve_io1`, `serve_io2`, `serve_vic`, `serve_128`
  - ROM banking: `bank_bits` with `rom_mode` (`00` = 8K, `01` = 16K, `11` = 32K banks), base `g_rom_base`, up to `g_max_cart_bits` (20 = 1 MB)
  - RAM mapping: `addr_map` (ROM/RAM/GEO) and `allow_write`; 64K cart RAM window at `g_ram_base`
  - C64 read-back: `slot_resp.data` and `slot_resp.reg_output`
- Freezer hooks are `freeze_trig`, `freeze_act`, `unfreeze`, `freezer_ena` and `hold_nmi`.

### Free logic codes
`cart_logic` is 5 bits. These codes are unused:
`10010`–`10111` (0x12–0x17), `11101` (0x1D) and `11110` (0x1E).
Proposal: **IDE64 = 0x12**, with `cart_variant` selecting the revision
(0 = V3, 1 = V4.1, 2 = V4.2).

### Slot interface limitation (needs a VHDL change outside `all_carts_v5`)
`fpga/cart_slot/vhdl_source/slot_slave.vhd` drives the C64 data bus on reads **only** in
these cases:
- IO1/IO2 asserted
- ROML/ROMH asserted
- kernal replacement

The address decode (`servicable`, around line 256) only covers `$8000-$FFFF`.
In ultimax mode, IDE64 maps **cart RAM at `$1000-$7FFF`** and ROM at `$A000-$BFFF`. The
C64 PLA gives no ROML/ROMH strobe for `$1000-$7FFF` (and not for `$A000-$BFFF` in
ultimax), so reads there are currently not served.
- **Writes are fine**: the write path (around line 192) captures any address when
  `allow_write='1'`.
- **Fix:** add a `serve_ultimax_lo` input (or a generic "serve this address" input) to
  `slot_slave`. Assert it in `servicable` for `$1000-$7FFF` and `$A000-$BFFF` while
  ultimax is active. Then drive `DATA_out` from `mem_data_0` in the data process when
  `dav='1'`.
- This also has to work in the U64 top level. That code isn't in this repo, so it's
  unknown whether the U64 uses `slot_slave` as-is.

### Software touch points
- `software/io/c64/c64.h`: `CART_TYPE_*` defines (the FPGA logic code plus variant bits).
- `software/io/c64/c64_crt.cc`: CRT hardware type 39 "IDE64" is listed as `CART_NOT_IMPL`.
- The code that maps `CART_*` to the logic type and variant, the cartridge menu/config lists,
  and the CRT chip loader (ROM image layout in the cart ROM area).

### Which targets can be built

| Target | Uses `all_carts_v5` | Bitstream buildable from this repo |
|---|---|---|
| U2+ (`target/fpga/u2plus_run`, `u2p_riscv`) | yes | yes (Quartus) |
| U2+L (`target/fpga/u2plus_ecp5`) | yes | yes (Lattice Diamond), but the device is nearly full |
| U64 / U64E-II / C64U | yes | **no**: the firmware embeds the prebuilt `external/u64e2_50t.bit` and `u64e2_100t.bit` |

- Commit `0c0fe7c1` rebuilt the U64E2 bitstreams after the COMAL 80 fix in
  `all_carts_v5.vhd`. So the U64E2 image does use this repo's cart VHDL, but its Vivado
  project, constraints and top level are **not** in the repo.
- Commit `22c3a167`: one extra fanout on a shared net was enough to make U2+L Diamond
  PAR fail. IDE64 **must** sit behind a generic that is off for U2+L.

## 2. IDE64 hardware spec (from VICE)

### Revisions

| | V3 | V4.1 | V4.2 |
|---|---|---|---|
| ROM size | 64K (4 × 16K banks) | 128K (8 banks) | 512K (32 banks) |
| Bank select | write `$DE32-$DE35` → bank = `(addr ^ 2) & 3` | write `$DE60-$DE67` → bank = `addr & 7` | write `$DE60-$DE7F` → bank = `addr & 31` |
| 16-bit data low byte | via `$DE30` latch | directly at `$DE20` | directly at `$DE20` |
| ID bits in `$DE32` read | `$10` | `$20` | `$80` |
| FT245 USB (`$DE5D-$DE5E`) | no | yes | yes |

- RAM is 32K on all revisions.
- VICE auto-detects the revision from the ROM: it scans `$1E60-$1EFC` for `STA $DE3x`,
  which means V3, or `STA $DE6x`, which means V4.
- Proposal: support **V4.1 first**, then V4.2 (only the bank width changes), and V3 last.

### Modes ($DEFB-$DEFF, write to select)
VICE `current_cfg` maps to CMODE values: 0 = 8K, 1 = 16K, 2 = off ("stnd"), 3 = ultimax ("open").

| Write | cfg | Mode | EXROM/GAME |
|---|---|---|---|
| `$DEFC` | 1 | 16K game | EXROM=0, GAME=0 |
| `$DEFD` | 0 | 8K game | EXROM=0, GAME=1 |
| `$DEFE` | 3 | ultimax | EXROM=1, GAME=0 |
| `$DEFF` | 2 | cart ROM off | EXROM=1, GAME=1 |
| `$DEFB` | — | "kill port": the written value is stored | bit0=1: cart disabled (falls through to cfg 2, off). bit1 = DS1302 CE line |

- **The modes apply to the CPU only (PHI2).** Every `cart_config_changed_slotmain()`
  call in `ide64.c` passes `0` (CMODE_8KGAME) as `mode_phi1`, the cfg as `mode_phi2`, and
  sets `CMODE_PHI2_RAM` ("VIC always sees RAM").
  - So the VIC never sees ultimax/16K. IDEDOS runs in ultimax without the VIC fetching
    cart ROM instead of the char ROM.
  - On the Ultimate: drive `game_n` from the table only while `phi2='1'`, and keep it
    `'1'` during PHI1. Set `serve_vic='0'`.
  - Precedent: `c_blackbox_v9` in `all_carts_v5` (`if phi2='1' then game_n <= ...`).
- **Reset state:** 8K mode, bank 0, kill_port 0, interface 0 (`ide64_config_init`).
- With kill bit0 set, the ROM window at `$DE60-$DEFF` and the RTC stop responding. In
  VICE the IDE registers and `$DE30-$DE37` still respond.
- Only reset leaves the killed state: writing `$DEFB` with bit0=0 only updates the RTC lines.

### C64 memory map per mode
ROM offset = `bank * $4000 + (addr & $3FFF)`.

| Range | 8K | 16K | Ultimax |
|---|---|---|---|
| `$1000-$7FFF` | C64 | C64 | **cart RAM**, offset `addr & $7FFF`, read/write |
| `$8000-$9FFF` | ROML | ROML | ROML |
| `$A000-$BFFF` | C64 | ROMH | **ROM** (offset `$2000-$3FFF`) |
| `$C000-$CFFF` | C64 | C64 | open bus in VICE (`ultimax_c000_cfff_read`) |
| `$E000-$FFFF` | C64 | C64 | ROMH (offset `$2000-$3FFF`) |

- ROM writes are ignored (`ide64_rom_store` is empty). VICE does not emulate flash
  programming.
- RAM `$0000-$0FFF` of the 32K chip is never reachable in VICE.
- The fast-path `ide64_mmu_translate` maps `$C000-$CFFF` to RAM offset `$4000`. This
  conflicts with the slow path, so check it on real hardware or with the IDE64 docs. Use
  open bus first.
- Mapping onto the Ultimate:
  - `rom_mode="01"` (16K banks), with `bank_bits(18 downto 14)`.
  - ROML/ROMH come from `slot_addr(13)`. That already gives the right offsets for
    `$8000`, `$A000` and `$E000`.
  - RAM goes to `g_ram_base + (addr & $7FFF)`.
  - Ultimax `$1000-$7FFF` and `$A000-$BFFF` reads need the `slot_slave` change described in §1.
  - The `$DE60-$DEFF` ROM mirror comes for free. For `$DExx`, `slot_addr(12 downto 0)` is
    already `$1Exx` and bit 13 is 0, so the existing `rom_addr` calculation gives
    `bank*$4000 + $1E60..$1EFF`.
  - `serve_io1='1'` would make *every* unclaimed `$DExx` address return ROM bytes
    (`$DE00-$DE1F`, `$DE38-$DE5C`). Gate serving to `$DE60-$DEFF` and the claimed
    registers (`reg_output`).
- **IO1 is owned entirely by IDE64.** The Ultimate's own IO1 users (ACIA at `$DExx`, etc.)
  must be off while it's active. IO2 stays free, so the REU and the command interface at
  `$DFxx` can coexist with IDE64.

### IO map ($DExx, all on IO1; IO2 unused)

| Address | Read | Write | Rev |
|---|---|---|---|
| `$DE00-$DE0F` | clockport device | clockport device | — (skip) |
| `$DE20-$DE27` | ATA CS0 regs 0–7 (see below) | ATA CS0 regs 0–7 | all |
| `$DE28-$DE2F` | ATA CS1 regs 8–15 | ATA CS1 regs 8–15. A write to `$DE28`/`$DE29` also selects interface 0/1 (`idrive = (addr & 1) << 1`) | all |
| `$DE30` | V3: low byte of the latched 16-bit data | V3: low byte of the 16-bit write latch | V3 |
| `$DE31` | high byte of the last 16-bit ATA read | high byte of the next 16-bit ATA write | all |
| `$DE32` | status: `ID \| bank<<2 \| ((cfg&1)^1)<<1 \| cfg>>1` | V3 bank select (`$DE32-$DE35`) | all |
| `$DE40-$DE4F` | ShortBus (DigiMAX, ETFE) | | — (skip) |
| `$DE5D-$DE5E` | FT245 USB: `$DE5D` data, `$DE5E` status (the handler switches on `(addr & 1) ^ 1`). With no USB connected, status reads `$C0` (bit6 = no RX data, bit7 = TX full) | `$DE5D` data | V4+ |
| `$DE5F` | DS1302: bit0 = data out (other bits are open bus) | DS1302: bit0 = data in. Each access is one SCLK pulse | all |
| `$DE60-$DEFF` | ROM mirror: `bank * $4000 + $1E00 + (addr & $FF)` (= `$9E60-$9EFF` of the bank) | `$DE60-$DE7F` bank select (V4.x), `$DEFB-$DEFF` config | all |

**16-bit data protocol (V4.1/V4.2)**
- *Read:* reading `$DE20` returns the low byte and latches the high byte. Then reading
  `$DE31` returns the high byte.
- *Write:* write `$DE31` with the high byte first. Then writing `$DE20` with the low byte
  stores the 16-bit word.
- Registers 1–15 are 8-bit; only the low byte matters.
- V3 uses `$DE30` for the low byte instead. In V3, `$DE20` reads are open bus (only the
  latch is updated).
- **The FPGA must handle this within the cycle**, because reading `$DE20` pops a word
  from the sector buffer.

### ATA registers (`ata.c`)

| Reg | `$DExx` | Read | Write |
|---|---|---|---|
| 0 | `$DE20` | data (16-bit, auto-incrementing buffer pointer while DRQ) | data |
| 1 | `$DE21` | error | features |
| 2 | `$DE22` | sector count | sector count |
| 3 | `$DE23` | sector / LBA 7:0 | same |
| 4 | `$DE24` | cylinder low / LBA 15:8 | same |
| 5 | `$DE25` | cylinder high / LBA 23:16 | same |
| 6 | `$DE26` | device/head (bit4 = dev, bit6 = LBA, 3:0 = head / LBA 27:24) | same |
| 7 | `$DE27` | status | command |
| 14 | `$DE2E` | alternate status | device control (bit2 = SRST, a 1→0 edge resets) |
| 15 | `$DE2F` | drive address: `$7F ^ (head<<2) ^ (1<<dev)` | — |

- **Status bits:** BSY `$80`, DRDY `$40`, DRQ `$08` (while `bufp < sector_size`), ERR `$01`.
- **Error bits:** UNC `$40`, IDNF `$10`, ABRT `$04`.
- **While BSY**, reads of registers 1–6 return status, as reg 14 does.
- A selected device that is absent leaves the bus floating (returns `bus`).
- **4 drives:** interface 0/1 × master/slave. Interfaces are selected via `$DE28`/`$DE29`,
  devices via reg 6 bit4. Start with 1 drive (interface 0 master).

**ATA commands VICE implements (`ata_execute_command`)**

| Group | Commands |
|---|---|
| Must-have | `$EC` IDENTIFY DEVICE · `$20`/`$21` READ SECTORS · `$30`/`$31` WRITE SECTORS · `$91` INITIALIZE DEVICE PARAMETERS · `$90` EXECUTE DIAGNOSTIC · `$EF` SET FEATURES (subcmds `$02`, `$03`, `$33`, `$55`, `$82`, `$99`, `$AA`) |
| Easy | `$00` NOP · `$40`/`$41` VERIFY · `$70` SEEK · `$E7` FLUSH CACHE · `$E4` READ BUFFER · `$E8` WRITE BUFFER |
| Power (fake: accept, set status) | `$94`–`$99`, `$E0`–`$E6` |
| ATAPI (CD-ROM) | skip |

- CHS and LBA28 addressing. IDENTIFY data comes from `ata_set_command_block` /
  `ata_poweron` in `ata.c`, which shows the exact words to copy.
- Image geometry: cylinders 1–65535, heads 1–16, sectors 1–63. VICE auto-detects it from
  the `.hdd` image (`detect_ide64_image` in `ide64.c`).

### RTC (DS1302 at $DE5F)
- **Lines:** CE = `$DEFB` bit1. Every `$DE5F` access pulses SCLK low→high.
  - Write: data in = bit0.
  - Read: bit0 = data out.
- **Protocol:** serial command byte, then data; registers in BCD. See `ds1202_1302.c`.
  31 bytes of battery-backed RAM; VICE optionally saves them.

### CRT format (`ide64_crt_attach`)
- Hardware type 39.
- Chips are always `$8000`, `$4000` bytes, with bank 0–31.
- 4 chips = 64K (V3), 8 chips = 128K (V4.1), 32 chips = 512K (V4.2).
- A raw `.bin` of 64K, 128K or 512K is also accepted.

## 2b. Resource budget (U2+)

- **Device:** U2+ runs on an EP4CE22F17C8 (22,320 LEs, 66 M9K blocks), with top level
  `u2p_nios_solo` (`fpga/fpga_top/ultimate_fpga/vhdl_source/u2p_nios2_solo.vhd`).
- **It is already close to full.**
  - Commit `1dea8f00` "Making it fit for now" (2023-04) had to drop `g_8voices`.
  - Commit `a20abb4b` then removed the tape-audio low-pass filters to get it back.
  - Since then, ACIA, RMII and 4 MB cart support have been added.
- **Utilization is unknown.** There's no fit report in the repo, and only the Quartus
  programmer is installed locally.
  - **First step:** install Quartus Prime Lite 18.1 (the version in the `.qsf`) and run a
    baseline fit of `target/fpga/u2plus_run`.
  - The U2+ firmware is NIOS II. It needs the Nios II EDS toolchain from the same Quartus
    release; `docker/Dockerfile` doesn't include it.

| Part | LEs (estimate) | M9K |
|---|---|---|
| `c_ide64` branch in `all_carts_v5` (modes, kill, 5-bit bank, `$DE32` read-back) | 100–200 | 0 |
| `slot_slave` ultimax serving | 20–40 | 0 |
| `ide64_ata` (task file, 16-bit latch, pointer, status, CPU regs, read muxes) | 300–500 | 1 (512 B buffer) |
| DS1302 + FT245 stub | 30–150 | 0 |
| **Total** | **~450–900 (2–4 % of the device)** | **1** |

**Measured baseline (2026-11-10).** Unmodified `u2plus_run`, Quartus 18.1 Lite, built
with `build_u2plus_fpga.sh` from the user's own terminal (the sandboxed session can't run
Quartus's IP generation):

| | Used | Free |
|---|---|---|
| Logic elements | 21,848 / 22,320 (**98 %**) | **472** |
| M9K | 56 / 66 | 10 |
| 9-bit multipliers | 16 / 132 | 116 |
| Pins | 143 / 154 | 11 |
| Worst setup slack | 0.328 ns (DDR2 PHY `pll1 clk[2]`) | — |

There are no critical warnings.

| Block (fitter, per entity) | LEs | M9K | 9-bit mult |
|---|---|---|---|
| `sid_peripheral` (UltiSID) | 2,096 | 13 | 10 |
| `sampler2` | 2,135 | 3 | 2 |
| `mm_drive` × 2 (second drive = `g_dual_drive`) | 3,337 + 3,344 | 2 + 2 | 0 |
| `all_carts_v5` (all existing carts) | 573 | 2 | 0 |
| `acia6551` | 299 | 1 | 0 |
| `reu` | 360 | 0 | 0 |

**Measured without UltiSID** (`g_stereo_sid => false` in `u2p_nios2_solo.vhd`, everything
else unchanged):

| | Baseline | No SID | Free after |
|---|---|---|---|
| Logic elements | 21,848 | 20,302 (91 %) | **2,018** |
| M9K | 56 | 43 | **23** |
| 9-bit multipliers | 16 | 6 | 126 |
| Worst setup slack | 0.328 ns | 0.499 ns | — |

There are no critical warnings. The saving is 1,546 LEs, less than the 2,096 the
per-entity report attributes to SID, because the fitter packs logic differently.

**Conclusion:** IDE64 (~450–900 LEs, 1 M9K) does **not** fit as-is. With UltiSID removed,
2,018 LEs and 23 M9K are free (measured), which is plenty. Run timing again after adding
IDE64; the setup margin is small.

**Decision (user): the UltiSID emulation may be removed for IDE64 builds.**
- Set `g_stereo_sid => false` in `u2p_nios2_solo.vhd`. Through `ultimate_logic_32` that
  sets `g_implement_sid = false` in `slot_server_v4`, which swaps `sid_peripheral` (16
  voices, waveform tables and filters) for an `io_dummy`.
- This is a supported configuration: `u2p_riscv.vhd` and the tester tops already build
  without SID.
- The firmware hides the SID options when `CAPAB_STEREO_SID` (capability bit 17) is clear;
  see `software/io/audio/audio_select.cc`.
- `g_8voices` then has no effect.
- This frees much more than IDE64 needs. Confirm by comparing the baseline fit with
  SID on and off.
- Bonus: UltiSID can be mapped to `$DExx`, which would clash with IDE64 owning IO1. With
  no SID, that conflict is gone too.

If it still doesn't fit:
- **Put the sector buffer in SDRAM instead of an M9K.** Put it at `g_ram_base + $8000`,
  above IDE64's 32K of RAM. A `$DE20` read becomes a normal cart memory fetch at
  `buffer + ptr`, and `slot_slave` already fetches 16 bits (`mem_data_0`/`mem_data_1`),
  which matches the 16-bit data latch. The firmware writes sectors directly to DRAM, with no
  copy through IO registers.
- **Keep the RTC in firmware.** The FPGA latches only CE/SCLK/IO, and the CPU shifts the
  bits.
- **For test builds, turn off something else:** `g_8voices`, `g_sampler`, `g_acia`, or
  the second drive.

## 2c. PCLink

Sources:
- the IDEDOS custom build page (https://singularcrew.hu/idedos/beta/compile)
- the IDEserv 0.35i source and README (`ideserv-latest.zip` from singularcrew.hu/idedos)
- VICE `ide64.c` (`usb_receive` / `usb_send` / `usbserver_activate`)

### Link options in IDEDOS, and what the Ultimate could offer

| IDEDOS PCLink option | Base | ideservd mode | Ultimate route |
|---|---|---|---|
| **USB via FT245 FIFO** | `$DE5D` (built into V4.1/V4.2) | `-m vice -i <ip> [-N 64245]`: **TCP client** to "VICE USB server" | **Emulate the FT245 at `$DE5D/$DE5E` and run a TCP server on port 64245** |
| SwiftLink serial | `$D700`, `$DE00`, `$DE10`, `$DF00` | `-m rs232s -d COMx` (38.4 kbit/s) | dropped (see option B) |
| Turbo232 serial | `$DE00`, `$DE10`, `$DF00` | `-m rs232 -d COMx` | the ACIA has a "Turbo232" hardware mode, but the speed is still limited |
| RR-Net / (E)TFE Ethernet | `$DE00`, `$DE10`, `$DF00` | `-m eth -i <if> -N n`: UDP port `64000+n`, broadcast queries, multicast replies to `239.0.64.n` | CS8900A emulation and raw Ethernet bridging |
| ETH64 / ETH64 II | `$DE00`, `$DE10` | `-m eth` | different chip, not considered |
| X1541 / PC64 / DUART / SilverSurfer | — | — | not applicable |

### Option A: FT245 + TCP server (recommended)
- **Native to V4.1.** IDEDOS for V4.1 has the FT245 at `$DE5D`, and ideservd already
  speaks to it over TCP (`-m vice`). That is exactly how VICE does it.
  - The bridge is byte-transparent. VICE just copies FIFO bytes into a TCP stream, with
    no framing.
  - **No PC-side changes:** `ideservd -m vice -i <ultimate-ip>`.
- **FPGA:** two byte FIFOs plus a status register, inside IDE64's own IO range
  (no extra address space).
  - **Read `$DE5D`:** pop an RX byte.
  - **Write `$DE5D`:** push a TX byte.
  - **Read `$DE5E` (status):** bit6 = 1 when RX is empty, bit7 = 1 when TX is full.
    `$C0` means no link.
  - **Reuse:** `fpga/io/acia/vhdl_source/acia6551.vhd` already has 256-byte rx/tx rings
    in one dual-port RAM, CPU-side head/tail pointers and IRQs. Add an "FT245 mode"
    there (different C64-side decode, no baud-rate throttling), or copy the ring logic
    into `ide64_ata`.
- **Firmware:** a TCP listener on port 64245 using lwIP. `software/io/acia/listener_socket.cc`
  is a starting point.
  - **RX:** socket → C64 RX ring.
  - **TX:** C64 TX ring → socket, flushed when full or on a short latency timer.
    VICE uses `LATENCY_TIMER 4000` cycles.
  - Accept one client at a time, and drop the buffers on disconnect, as VICE does.
- **Speed:** no baud limit, so it's limited by the C64-side loop. TCP also makes it
  reliable over Wi-Fi and routed networks.
- **IDEDOS build:** choose PCLink "USB via FT245 FIFO" with base `$DE5D`.
- **PC client: no new version needed.** I checked the ideservd 0.35i `src/vice.c`. It is
  a plain TCP client with nothing VICE-specific:
  - `-i` takes any IP or hostname; `-N` sets the port (default 64245).
  - It sets `TCP_NODELAY` and sends no handshake or framing.
  - If the connection is refused, it logs "Waiting for VICE to show up" and retries.
  - The Windows `ideservd.exe` in the zip includes `-m vice`.
  - The only oddity is cosmetic: logs say "VICE".
- **What the Ultimate side must honour:**
  - **Prompt TX flush (C64 → PC):** ideservd gives up after **1000 ms** without a byte
    in mid-transfer (`refill(1000)`). Use a latency timer of a few ms, as VICE does.
  - **Never drop bytes:** stop reading the socket while the C64 RX FIFO is full, and let
    TCP flow control apply backpressure.
  - **Disconnect:** clear both FIFOs and accept the next client.
  - **Port:** make it configurable, default 64245.

### PC side: a Windows GUI for ideservd (optional)
These findings come from the ideservd 0.35i source.

**Current state:**
- Windows is already a native target: `Makefile.mingw32` (`i686-w64-mingw32-gcc`,
  `-mwindows`, `-DHAVE_USB -DHAVE_WS2 -DHAVE_COMCTL`, `windres` for `resource.rc`).
- The GUI is minimal. `resource.rc` defines `IDD_MAIN` with only a log edit box and two
  buttons ("Hide", "Close ideservd"), plus a tray icon (`Shell_NotifyIcon`).
- All settings come from the command line (`init_arguments` in `arguments.c`). The `.bat`
  files in the zip are how users configure it today.
- `main()` (around line 970 of `ideservd.c`) does the following:
  1. creates the dialog;
  2. parses the arguments and runs `get_driver()` and `prepare_environment` /
     `setup_environment`;
  3. starts `PCLinkLoop` in a worker thread;
  4. stops it with `QueueUserAPC(PCLinkTerminate)`, which sets `signal_received`.

**Licence:** GPL v2 (`COPYING`). A modified build is allowed, but its source must be
shipped with it. Offer the change upstream to Kajtar Zsolt (soci) rather than forking.

**Proposed change (small and contained, Windows only):**
1. **Extend `IDD_MAIN` in `resource.rc`:**
   - "Ultimate host/IP" edit box and "Port" edit box (default 64245)
   - "Shared folder" edit box with a Browse… button (`SHBrowseForFolder`)
   - Start/Stop button
   - optionally a mode combo (vice/usb/eth/rs232), so the GUI isn't limited to the
     Ultimate
2. **Persist the settings** to an INI file next to the exe, or under
   `HKCU\Software\IDE64\ideservd` (`GetPrivateProfileString` / `WritePrivateProfileString`).
   With no command-line arguments, start from the saved settings; arguments still
   override them.
3. **Apply changes by restarting the server.** The simplest robust way is for the exe to
   relaunch itself with the new arguments (`CreateProcess` + exit). The resolved address
   in `vice.c` `initialize()` is cached in a function-static `sockaddr_in`, and
   `prepare_environment` / `setup_environment` run once. An in-process restart would
   require resetting those, plus `signal_received`, `buffers_*` and `partition_*`.
4. **Optional:** a "Test connection" button that does a plain TCP connect to host:port and
   reports the result.

**Build:** use MinGW-w64. On Linux or WSL: `make -f Makefile.mingw32`. On Windows with
MSYS2 (`C:\msys64\mingw64\bin\gcc.exe` is installed on this machine):
`make -f Makefile.mingw32 CC=gcc`.
- The Makefile assumes the 32-bit `i686` cross compiler. A 64-bit build needs
  `CC=gcc` and `windres` from the mingw64 environment.
- Check `ftd2xxl.c`, `ws2.c` and `iphlpapil.c` (runtime-loaded DLL wrappers) for any
  32-bit assumptions.
- **Visual Studio port (assessed, not done).** VS 2019, 2022 and 18 are installed
  locally. The port is small and doesn't touch the protocol code:
  - **Already MSVC-safe:** `__attribute__` sits behind `#ifdef __GNUC__`, there are no
    VLAs or statement expressions, and `getopt` is bundled (`my_getopt.c`). The DLLs
    (ws2_32, iphlpapi, ftd2xx) are loaded at runtime, so there are no import
    libraries to find.
  - **Breaks under MSVC:** unconditional `<unistd.h>`, `<sys/time.h>` and `<strings.h>`
    (in `compat.c`, `directory.c`, `ideservd.c`, `normal.c`, `usb.c`, `log.c`,
    `driver.c`). The POSIX calls used are `gettimeofday`, `usleep`, `strcasecmp`,
    `ftruncate`, `chdir`, `getcwd`, `access`, `S_ISDIR`/`S_ISREG`, `ssize_t` and `off_t`.
    **Fix:** a `msvc_compat.h` force-included with `/FI` under `#ifdef _MSC_VER`, plus
    `#ifndef _MSC_VER` around those three includes.
  - **Parallel-port drivers are not needed (user decision):** X1541/XE1541/XM1541/XA1541
    and PC64 are dropped.
    - Leave `parport.c`, `x1541.c` and `pc64.c` out of the VS project. Gate them with a
      define (e.g. `NO_PARPORT`) in `parport.h`, `x1541.h`, `pc64.h` and `driver.c`, so
      `-m x1541`/`pc64` report "No such driver".
    - That also removes the GCC inline asm `inb`/`outb` in `parport.h` and the inpout32
      DLL dependency.
    - The MinGW build keeps them, so the upstream behaviour is unchanged.
  - **Drivers kept in the VS build:** `vice` (TCP, used with the Ultimate), `usb` (real
    IDE64 V4.1 owners), `eth`, and `rs232` (plain Win32 comm API, no extra cost).
  - **Project settings:**
    - C11 (`/std:c11`, for designated initializers in the driver tables)
    - `/SUBSYSTEM:WINDOWS` with `/ENTRY:mainCRTStartup`, because the code uses `main()`
    - defines `HAVE_USB;HAVE_WS2;HAVE_COMCTL;_CRT_SECURE_NO_WARNINGS`
    - character set "Not Set" (the code calls the W and A APIs explicitly)
    - link `winmm.lib` and `comctl32.lib`
    - `/MANIFEST:NO`, because `resource.rc` embeds `resource.xml`
  - Keep the MinGW makefiles working, so the patch is acceptable upstream.
- Not yet verified: the first local build attempt failed because gcc couldn't create a
  temporary file (the build environment pointed it at `C:\WINDOWS\`). The second attempt
  was blocked by the session's permission policy.

### Option B: SwiftLink: dropped
Dropped by the user (2026-11-10): too slow (38.4 kbit/s), it needs a virtual COM port on
the PC, and it's no use to the U64E-II. The ACIA/modem code is not touched.

### Option C: RR-Net (CS8900A) emulation (largest; separate project)
- **C64 side:** the CS8900A packet-page model. `fpga/io/cs8900a/vhdl_source/cs8900a_bus.vhd`
  is an unfinished 2022 stub (it references undeclared signals and has no packet RAM),
  is not instantiated anywhere, and has no software side.
- **Network side:** the C64 builds raw Ethernet, ARP, IP and UDP frames itself. The
  Ultimate would have to bridge raw frames with a second MAC address onto its LAN:
  - receive broadcast and multicast `239.0.64.n`
  - inject frames past lwIP
- On Wi-Fi-only units it's harder still, and the PCLink UDP protocol has no
  retransmission.
- **Pro:** it also gives general RR-Net networking to other C64 software. That's
  worthwhile on its own, but it's a much bigger job than IDE64 PCLink needs.

**Decision:** do A in Phase 4. B is dropped. Track C as a separate, independent feature.

## 3. Open decisions
- [x] **Targets (user decision, 2026-11-10):** develop and prove it on the **U2+**
      first. The **main target is the U64E-II**, and the **U64** is in scope too. U2+L is
      out.
- [ ] **U64E-II/U64 FPGA source.** We build the test images ourselves, but the Artix-7
      Vivado project (top level, C64 core, constraints) is not in this repo. Get it, or a
      way to build it, from GideonZ or the contributor behind `0c0fe7c1`. Ask in the
      issue. This blocks Phase 5 for U64, not the U2+ work.
- [x] Revision for the first version: **V4.1**, with V4.2 as a small extension.
- [x] Disk backend: **an image file** (`.hdd` / `.cfa`, the same as VICE). Rationale:
  - **Existing code:** `software/filesystem/blockdev_file.{h,cc}` (`BlockDevice_File`)
    already turns a `File` into a sector device. It's used by `embedded_fat` and
    `embedded_iso`, so the ATA handler is simply `seek(lba << 9)` + `read/write(512 * n)`.
  - **Portable:** images move between the Ultimate and VICE unchanged, and can be backed
    up and swapped from the file browser.
  - **No risk to the Ultimate's own FAT volume.** With a raw partition, the C64 would
    write CFS straight onto the medium the Ultimate itself mounts. It would also need a
    partition-table type, and USB sticks would have to be dedicated to IDE64.
  - **Geometry:** use `detect_ide64_image` in VICE's `ide64.c`:
    - a `"C64-IDE V"` header at 0 means old CHS;
    - a `"C64 CFS V"` header at 8 means LBA or CHS taken from bytes 4–7;
    - otherwise, size / 512 means LBA.
  - **Size limits:** FAT32 caps an image at 4 GB. exFAT is enabled (`FF_FS_EXFAT 1` in
    `chan_fat/full/ffconf.h`), so larger images work there. IDE64 needs far less anyway.
  - **Seek cost:** `FF_USE_FASTSEEK` is 0, so a backward seek in a big image walks the
    cluster chain. If that's slow, enable fast seek for this file (a cluster link map
    table), or recommend large cluster sizes. Images can't grow while mounted, so
    preallocate them at creation.
  - **Possible later option:** a raw-partition backend is the same `BlockDevice`
    interface, so it can still be added.
- [x] RTC: real clock from the Ultimate's RTC. The 31 bytes of RTC RAM (IDEDOS setup)
      are stored in a sidecar file next to the mounted image (`<image>.rtc`), so the
      settings follow the disk image. Writes are debounced (save a few seconds after the
      last change, and on reset/detach). (Decided 2026-11-11.)
- [ ] Ultimax `$C000-$CFFF`: open bus (VICE slow path) or RAM (VICE mmu_translate).

## 4. Plan

### Phase 0: preparation
- [x] Extract the register and memory map from VICE (§2).
- [ ] Get an IDEDOS ROM (`.crt` V4.1) and a test `.hdd` image that work in VICE.
- [ ] For U64E-II: ask how the U64E2 bitstream is built (Vivado project, version).

### Phase 1: VHDL, banking and modes only (no disk)
Status (2026-11-12): implemented, simulated, host-tested and built for U2+ (fits, timing
met); **not yet run on hardware**. Nothing is committed yet.
- [x] `g_ide64 : boolean := false` generic on `all_carts_v5`, `slot_server_v4` and
      `ultimate_logic_32`; capability bit 20 (`CAPAB_IDE64`, the old unused
      `CAPAB_OVERLAY` bit) reports it to the firmware.
- [x] U2+ IDE64 build: `u2p_nios2_solo.vhd` has `g_ide64 : boolean := true` and
      `g_stereo_sid => not g_ide64`. **Local only:** the stock image must keep
      `g_ide64 => false` (flip the default before any upstream PR).
- [x] `constant c_ide64 := "10010"` and its `when c_ide64` branch:
  - `$DEFB-$DEFF` mode and kill handling, `$DE60-$DE67` (V4.1) / `$DE60-$DE7F` (V4.2)
    bank select into `bank_bits(18 downto 14)`
  - GAME driven only while `phi2='1'`, EXROM from cfg, `serve_vic='0'`
  - `rom_mode="01"`, `serve_rom`, `serve_io1` only for `$DE60-$DEFF` (and not when killed)
  - placeholder IDE bus latch at `$DE20-$DE2F`/`$DE31` (no drive: reads return the last
    bus value, as in VICE); `$DE5E` FT245 status reads `$C0`
- [x] `slot_resp`: `$DE32` status (ID `$20`/`$80`, bank, cfg), `$DE20-$DE2F`, `$DE31`, `$DE5E`.
- [x] `addr_map`: ultimax `$1000-$7FFF` → cart RAM (`addr & $7FFF`) with `allow_write`.
      The `$DE60-$DEFF` ROM mirror comes from the normal ROM path.
- [x] `slot_slave`: new `serve_ultimax` input; serves CPU reads (PHI2, RWn=1) of
      `$1000-$7FFF` and `$A000-$BFFF` from `mem_data_0`. Passed through `slot_server_v4`.
      Other carts never assert it.
- [x] Testbench `fpga/cart_slot/vhdl_sim/tb_ide64.vhd` (`slot_timing` + `slot_slave` +
      `all_carts_v5`, PLA model, memory model): modes and GAME/EXROM per half cycle,
      banking (V4.1 and V4.2), `$DE32`, ROM window, unclaimed IO1 left open, ultimax RAM
      read/write and offsets, ultimax `$A000`/`$E000`, kill and reset. **Passes** in
      ModelSim ASE 18.1 (`vcom -2008`; compile order: `io_bus_pkg`, `slot_bus_pkg`,
      `io_dummy`, `fpga/altera/dpram.vhd`, `microwire_eeprom`, `slot_timing`, `slot_slave`,
      `all_carts_v5`, `tb_ide64`).
- [x] Software:
  - `CART_TYPE_IDE64 0x12` in `c64.h`, `CAPAB_IDE64` in `itu.h`
  - CRT type 39 → `CART_IDE64`: refused when the FPGA lacks `CAPAB_IDE64`; V3 ROMs
    (VICE's `STA $DE3x` scan of `$1E60-$1EFC`) refused; more than 8 banks → V4.2
    (`VARIANT_2`), else V4.1 (`VARIANT_1`); `prohibit = CART_PROHIBIT_DEXX`
  - chips (`$8000`, 16K) land at `bank * $4000` through the existing loader
  - host tests in `software/io/c64/tests` (types, refusals, V3/V4 detection, placement):
    560 checks pass. On this machine, build with MSYS2
    `make CXX=/c/msys64/usr/bin/g++ CXXFLAGS="-std=gnu++14 ..."` (plain `c++14` hides
    `strcasecmp` on MSYS2).
- [ ] Raw `.bin` ROM (64K/128K/512K) menu entry: deferred; convert with VICE `cartconv`.
- [x] Build the U2+ FPGA and firmware (2026-11-12, `build_u2plus.cmd`). Phase 1 costs
      75 LEs over the no-SID numbers in §2b:

      | | No SID | No SID + IDE64 Phase 1 |
      |---|---|---|
      | Logic elements | 20,302 | 20,377 (91 %, 1,943 free) |
      | Worst setup slack | 0.499 ns | 0.448 ns (DDR2 PHY `pll1 clk[3]`) |
      | Worst hold slack | — | 0.398 ns (`ref_clock`) |

- [x] Windows packaging: `build_u2plus.cmd [fpga|update]` runs `build_u2plus_fpga.sh` and
      `build_u2plus_update.sh`. The latter builds `tools/*.exe` with MSYS2 gcc and passes
      them as make overrides, because Git Bash runs the extensionless Linux ELF
      `tools/hex2bin` ahead of `hex2bin.exe`.
- [ ] Milestone: IDEDOS boots and shows its banner. With no drive it should report that
      no device was found.
- **First hardware checks (not covered by the testbench):**
  - GAME switching per half cycle: the testbench PLA reacts to GAME with a fixed delay,
    not to real PHI2 edge timing. If GAME arrives late in PHI2, C64 DRAM may drive
    `$1000-$7FFF` before the cart does. Put a logic analyzer on GAME vs PHI2 first.
  - Kernal replacement vs ultimax ROMH at `$E000-$FFFF`: test with kernal replacement
    off. Then check which one wins in `slot_slave`'s data process, or have the firmware
    force kernal replacement off while the cart is IDE64.

### Phase 2: ATA device in the FPGA
- [ ] New entity `fpga/cart_slot/vhdl_source/ide64_ata.vhd`:
  - **C64 side** (`$DE20-$DE31`): the task file registers, the 16-bit latch semantics,
    and the data-pointer increment on a `$DE20` read or write.
    - While BSY, regs 1–6 read as status.
    - DRQ = pointer < 512.
    - SRST via reg 14.
    - Interface select via `$DE28`/`$DE29`.
  - **CPU side:** an `io_bus` register block: the task file snapshot, a "command written"
    flag plus IRQ, a status/error write-back, and an "unknown device" flag per drive.
  - a 512-byte dual-port sector buffer (block RAM)
- [ ] Hook it into `slot_server_v4` through a free `i_split_8K` slot (`0x404C000` or above),
      under `g_ide64`. ACIA (`io_req_acia` at `0x404A000`) is the template.
- [ ] GHDL testbench under `fpga/cart_slot/vhdl_sim` that drives C64 bus cycles and CPU
      accesses.

### Phase 3: firmware ATA emulation
- [ ] New driver in `software/io/c64/` (e.g. `ide64.cc`). On the command IRQ:
  - decode the command (must-have list in §2), handle CHS/LBA
  - fill or drain the buffer per sector, then clear BSY / set DRQ or ERR
  - multi-sector transfers refill the buffer each time DRQ drops
  - IDENTIFY words follow `ata.c`
- [ ] Mount a `.hdd`/`.cfa` image from the file browser through `BlockDevice_File` (a
      new filetype handler). Detect the geometry as in `detect_ide64_image`, add a config
      item for the image path, and auto-mount it at boot.
- [ ] A "create new IDE64 image" menu action that preallocates the file at a chosen size.
      IDEDOS's own setup tool can format it as CFS.
- [ ] Milestone: IDEDOS sees the drive, and a directory listing, a file load and a save work.

### Phase 4: RTC, USB stub and polish
- [ ] DS1302: CE/SCLK/IO in the FPGA as a shift register, or a small state machine.
      Time comes from the Ultimate's RTC, and the 31 bytes of RAM are persisted to
      `<image>.rtc` next to the mounted `.hdd`/`.cfa` (debounced writes).
- [ ] FT245 / PCLink (§2c option A):
  - byte FIFOs at `$DE5D`/`$DE5E` (status `$C0` when idle)
  - a firmware TCP server on port 64245 bridging to the FIFOs
  - test with `ideservd -m vice -i <ultimate-ip>`, or in VICE first, to check the IDEDOS
    build options
- [ ] Interaction with freeze/menu, `cart_kill`, and reset (back to 8K mode, bank 0).
- [ ] V4.2 (5-bit bank, 512K) and V3 (`$DE30` latch, `$DE32-$DE35` banking) variants.
- [ ] Check resources and timing on every target that has `g_ide64` on.
- [ ] Add e2e tests under `tests/` if the harness supports cart boot checks.

### FPGA build setup (U2+, local)
- **Quartus Prime Lite 18.1** at `C:\intelFPGA_lite\18.1`, the version the `.qsf` was
  last saved with. It includes:
  - `quartusin64\quartus_sh.exe`
  - Cyclone IV E device support (EP4CE22F17C8)
  - `nios2eds`
- **Build order** (the same as `make u2plus` in the top-level `Makefile`):
  1. `make -C target/u2plus/nios/boot_run`. Needs the Nios II gcc from `nios2eds`, and
     writes `target/fpga/u2plus_run/onchip_mem.hex`, the boot ROM baked into the FPGA.
     The 18.1 Nios II EDS on Windows runs its toolchain through WSL; Ubuntu is installed.
  2. `make -C target/fpga/u2plus_run`. Runs `quartus_sh --flow compile ultimate_run`
     and fails on any "Critical Warning". Output goes to `output_files/ultimate_run.rbf`.
  3. Build the firmware (`target/u2plus/nios/ultimate`, `updater`) to get `update.u2p`
     with the new `.rbf`.
- **First run:** an unmodified baseline build, to record LE/M9K usage (§2b). Then
  build with `g_stereo_sid => false` and compare.
- Make a separate IDE64 build variant (generic or separate top) so the stock image stays
  unchanged.

### Phase 5: target builds
- [ ] U2+: build with Quartus, test on hardware.
- [ ] U64E-II (main target), then U64: build the bitstream with the external Vivado
      project (Vivado ML Standard, which is free and covers XC7A50T/100T), replace
      `external/u64e2_*.bit`, bump `g_version` in `ultimate_logic_32.vhd`, then rebuild the
      firmware with `build_u64ii.cmd`.

## 5. Risks
- No U64E-II Vivado project in this repo means the U64 path is blocked without outside help.
- The ultimax RAM change in `slot_slave` touches shared bus timing for all carts. Keep it
  gated so other carts behave identically.
- U2+L's FPGA has almost no spare room, so IDE64 stays disabled there.
- GAME has to switch per half-cycle (low only in PHI2), so its timing relative to the PHI2
  edges matters. `game_n` is registered on the FPGA clock, and the PLA must see the
  ultimax config before the CPU address/data phase. Check on hardware with a logic
  analyzer. `c_blackbox_v9` is the only existing cart doing this.
- IDEDOS relies on exact ATA status behaviour. BSY latency from firmware servicing may
  expose edge cases, so keep the buffer, pointer and status logic entirely in the FPGA.
- Real IDE64 V4 has writable flash (IDEDOS updates). VICE doesn't emulate it, and neither will we.

## 6. Upstreaming (prepared, not submitted)

Nothing is pushed, and no issue or pull request is opened, until the user says so.

### Rules from `CONTRIBUTING.md` that apply here
- **Issue first.** This adds a feature, touches `fpga/` and is far over 300 lines, so it
  needs an issue and a go-ahead from the maintainer (@GideonZ) *before* most of the
  code. All VHDL goes through him, and he builds the FPGA images.
- **Contacts:**
  - REST API and tests: @chrisgleissner (he also built the U64E2 images in `0c0fe7c1`)
- **One problem per issue, one issue per PR.** Each PR must apply to `master`, build and
  be useful alone, with no chains. A PR that needs FPGA changes not yet on `master`
  stays a draft.
- **Code rules:**
  - LF endings; match the surrounding style; add files by name (never `git add -A`; the
    working tree has untracked build files)
  - mind the image size; `small_printf` has no `l`, `ll` or `z` modifiers
  - shared code stays generic (an IDE64 filetype is its own class)
- **Testing:**
  - red/green runs with `./run-tests` (suites in `tests/`, helpers in `tests/lib`)
  - the standard profile, plus the report from `tools/e2e_report.py`
  - a test on an Ultimate-II and on a U64/C64U. We can't do the U64 one without a
    bitstream, so say so.
- **PR description:** the problem plus the issue link, what changed and why, how it was
  tested (devices, firmware/FPGA versions, red/green results), and what's missing.
  Mention the model used (Claude Opus 5.5). Use draft status until ready.
- **Remote:** `origin` is `GideonZ/1541ultimate`. Pushing needs a fork remote (e.g.
  `git remote add fork https://github.com/<you>/1541ultimate.git`).

### Local branch
- `feature/ide64`, created from `master` at `9537fbc8`. It's for local work only. PR
  branches are cut from current `master` per step, so each PR stands alone.
- Keep these out of every commit:
  - `build_u64ii.cmd` and `build_u64ii.sh`
  - `u64ii/`
  - the stray `target/u64ii/riscv/update/.cc`
  - this plan file, unless the maintainer wants it in `doc/`. Otherwise its content
    goes into the issue.

### Proposed issue and PR split

| Step | Issue / PR | Content | Mergeable when |
|---|---|---|---|
| 0 | **Issue:** "IDE64 V4.1 cartridge emulation" | the text below | — |
| 1 | PR (FPGA, likely done or rebuilt by the maintainer) | `c_ide64` in `all_carts_v5`; ultimax `$1000-$7FFF`/`$A000-$BFFF` serving in `slot_slave`; `ide64_ata` + FT245 FIFO; all behind `g_ide64` (default false, so stock images are unchanged); GHDL testbench | after the issue's go-ahead |
| 2 | PR (software) | `CART_TYPE_IDE64`; CRT type 39 mapping; ATA emulation driver; `.hdd`/`.cfa` mount via `BlockDevice_File`; image creation; e2e test | after step 1 is on `master` and an image with `g_ide64` exists (draft until then) |
| 3 | **Issue + PR:** "PCLink over TCP for IDE64" | firmware TCP server on port 64245 bridging the FT245 FIFO; config item; test with `ideservd -m vice` | after step 2 |
| — | outside this repo | ideservd Visual Studio project and GUI fields: a patch sent to the ideservd author (Kajtar Zsolt, soci), keeping the MinGW makefiles working | independent |

### Issue draft (step 0)

> **Title:** IDE64 V4.1 cartridge emulation
>
> **Need:** IDE64 is a widely used C64 mass-storage and DOS cartridge (IDEDOS). It
> can't currently run on the Ultimate; CRT hardware type 39 is listed as
> `CART_NOT_IMPL` in `c64_crt.cc`.
>
> **Products:**
> - The main target is the **U64E-II** (and so the C64U). The U64 is in scope too.
> - Development and first hardware tests are on the **U2+**, because its FPGA builds
>   from this repo.
> - The VHDL goes into the shared `all_carts_v5`/`slot_server_v4`.
> - Not U2+L (no room).
>
> **Proposed approach** (VICE's `ide64.c`/`ata.c` is the reference):
> - **FPGA:** a new cart logic type (free code `0x12`) for modes, banking, the
>   `$DE60-$DEFF` ROM window and 32K RAM.
>   - `slot_slave` must serve ultimax reads at `$1000-$7FFF` and `$A000-$BFFF`, where
>     the PLA gives no ROML/ROMH strobe. This is gated so other carts are unaffected.
>   - GAME is driven only during PHI2, as on the real card (the VIC never sees ultimax).
>   - A small ATA register block with a 512-byte sector buffer, and the FT245 FIFO
>     at `$DE5D`.
>   - Everything sits behind a `g_ide64` generic, default off.
> - **Firmware:** ATA commands are served from a `.hdd` image file through
>   `BlockDevice_File`. PCLink goes over the V4.1 FT245 FIFO, bridged to a TCP server on
>   port 64245. That's the same protocol as VICE's IDE64 USB server, so stock IDEDOS and
>   stock `ideservd -m vice -i <ultimate-ip>` work unchanged.
> - **U2+ fit:** I'd drop UltiSID (`g_stereo_sid => false`) in an IDE64 build if space
>   is needed.
>
> **Questions:**
> 1. Is this approach OK, especially the `slot_slave` change? Does the U64/U64E-II top
>    level use `slot_slave` the same way, given that its C64 is internal?
> 2. We want to build U64E-II/U64 test bitstreams ourselves. Can you share the Vivado
>    project, or tell us how contributors build them (as for the COMAL 80 fix in
>    `0c0fe7c1`)?
> 3. Should IDE64 be in the standard images, or in a separate IDE64 build variant?
