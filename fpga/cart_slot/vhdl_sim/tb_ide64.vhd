-------------------------------------------------------------------------------
-- Title      : tb_ide64
-------------------------------------------------------------------------------
-- Description: Testbench for the IDE64 cart logic (c_ide64 in all_carts_v5)
--              together with slot_slave and slot_timing. A simple C64 bus
--              model (CPU cycles in PHI2, simplified PLA) and a memory model
--              holding the cart ROM and RAM. Checks modes, banking, the
--              $DExx registers, ultimax RAM serving and the kill port.
--              The ATA device (ide64_ata) is driven by a firmware model on
--              its io_bus: IDENTIFY, READ/WRITE SECTORS, absent devices,
--              BSY behaviour and SRST.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.io_bus_pkg.all;
use work.slot_bus_pkg.all;
use work.ide64_ata_pkg.all;

entity tb_ide64 is
end entity;

architecture tb of tb_ide64 is
    constant c_freq         : natural := 62_500_000;
    constant c_half_period  : time := 508 ns;
    constant c_rom_base     : std_logic_vector(27 downto 0) := X"0F00000";
    constant c_ram_base     : std_logic_vector(27 downto 0) := X"0EF0000";

    signal clock            : std_logic := '1';
    signal reset            : std_logic := '1';
    signal stop             : boolean := false;
    signal PHI2             : std_logic := '0';

    -- C64 bus
    signal ADDRESS          : unsigned(15 downto 0) := X"3FFF";
    signal RWn              : std_logic := '1';
    signal BA               : std_logic := '1';
    signal cpu_wdata        : std_logic_vector(7 downto 0) := X"00";
    signal cpu_drive        : boolean := false;
    signal bus_data         : std_logic_vector(7 downto 0);
    signal IO1n, IO2n       : std_logic := '1';
    signal ROMLn, ROMHn     : std_logic := '1';
    signal GAMEn, EXROMn    : std_logic := '1'; -- as seen on the bus (synchronized)
    signal game_line        : std_logic := '1';
    signal exrom_line       : std_logic := '1';

    -- slot slave / timing
    signal data_o           : std_logic_vector(7 downto 0);
    signal data_t           : std_logic;
    signal allow_serve      : std_logic;
    signal phi2_tick        : std_logic;
    signal clear_inhibit    : std_logic;
    signal do_sample_addr   : std_logic;
    signal do_sample_io     : std_logic;
    signal do_io_event      : std_logic;
    signal slot_req         : t_slot_req;
    signal slot_resp        : t_slot_resp;
    signal slot_resp_cart   : t_slot_resp;
    signal slot_resp_ide    : t_slot_resp;
    signal epyx_timeout     : std_logic;
    signal kernal_area      : std_logic;

    -- cart logic
    signal cart_logic       : std_logic_vector(4 downto 0) := "10010";
    signal cart_variant     : std_logic_vector(2 downto 0) := "001";
    signal c64_reset        : std_logic := '0';
    signal cart_kill        : std_logic := '0';
    signal cart_active      : std_logic;
    signal serve_enable     : std_logic;
    signal serve_vic        : std_logic;
    signal serve_128        : std_logic;
    signal serve_rom        : std_logic;
    signal serve_io1        : std_logic;
    signal serve_io2        : std_logic;
    signal serve_ultimax    : std_logic;
    signal allow_write      : std_logic;
    signal game_n           : std_logic;
    signal exrom_n          : std_logic;
    signal ide64_active     : std_logic;
    signal ide64_rtc_ce     : std_logic;
    signal ide64_killed     : std_logic;
    signal fw_rtc_time      : natural := 0;     -- DS1302 clock write events seen by the firmware model
    signal fw_rtc_ram       : natural := 0;     -- DS1302 RAM write events seen by the firmware model
    signal fw_ram0          : std_logic_vector(7 downto 0) := X"00"; -- RAM byte 0, read on the last RAM event
    signal fw_sec           : std_logic_vector(7 downto 0) := X"00"; -- seconds register, read on the last clock event
    -- FT245: the test asks the firmware model to act (1 = connect + receive "ABC",
    -- 2 = take the bytes sent by the C64, 3 = disconnect + flush)
    signal usb_cmd          : natural := 0;
    signal usb_kind         : natural := 0;
    signal usb_done         : natural := 0;
    signal usb_txn          : natural := 0;    -- bytes taken by the last kind 2
    signal usb_tx0          : std_logic_vector(7 downto 0) := X"00";
    signal usb_tx2          : std_logic_vector(7 downto 0) := X"00";

    -- ATA device, CPU side
    signal io_req           : t_io_req := c_io_req_init;
    signal io_resp          : t_io_resp;
    signal io_irq           : std_logic;
    signal fw_slow          : boolean := false; -- firmware model takes 30 us per step
    signal fw_writes        : natural := 0;     -- sectors received by WRITE SECTORS
    signal fw_resets        : natural := 0;     -- reset events acknowledged by the firmware model

    -- memory
    signal mem_req          : std_logic;
    signal mem_rwn          : std_logic;
    signal mem_rack         : std_logic := '0';
    signal mem_dack         : std_logic := '0';
    signal mem_rdata        : std_logic_vector(31 downto 0) := (others => '0');
    signal mem_wdata        : std_logic_vector(31 downto 0);
    signal mem_addr         : unsigned(25 downto 0);

    type t_ram is array(natural range <>) of std_logic_vector(7 downto 0);
    shared variable ram     : t_ram(0 to 65535) := (others => X"00");
    shared variable wbuf    : t_ram(0 to 511) := (others => X"00"); -- last sector written by the C64
    signal errors           : natural := 0;

    function rom_byte(i : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(((i mod 251) + 37 * (i / 16384)) mod 256, 8));
    end function;

    -- sector contents served by the firmware model (IDENTIFY uses lba 1000)
    function sec_byte(lba, i : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned((lba * 13 + i * 3 + i / 256) mod 256, 8));
    end function;

    function hex(v : std_logic_vector) return string is
        constant c : string(1 to 16) := "0123456789ABCDEF";
        variable r : string(1 to v'length/4);
        variable n : std_logic_vector(v'length-1 downto 0) := v;
    begin
        for i in r'range loop
            r(i) := c(1 + to_integer(unsigned(n(n'high - 4*(i-1) downto n'high - 4*(i-1) - 3))));
        end loop;
        return r;
    end function;
begin
    clock <= not clock after 8 ns when not stop; -- 62.5 MHz
    PHI2  <= not PHI2 after c_half_period when not stop;

    ---------------------------------------------------------------------------
    -- Bus: cart lines (open collector, delayed by the synchronizers) and a PLA
    ---------------------------------------------------------------------------
    game_line  <= '0' when cart_active='1' and game_n='0'  else '1';
    exrom_line <= '0' when cart_active='1' and exrom_n='0' else '1';
    GAMEn  <= transport game_line  after 40 ns;
    EXROMn <= transport exrom_line after 40 ns;

    p_pla: process(ADDRESS, RWn, game_line, exrom_line)
        variable v_umax  : boolean;
        variable v_roml  : std_logic;
        variable v_romh  : std_logic;
        variable v_io1   : std_logic;
        variable v_io2   : std_logic;
    begin
        -- HIRAM = LORAM = CHAREN = 1
        v_umax := game_line = '0' and exrom_line = '1';
        v_roml := '1';
        v_romh := '1';
        v_io1  := '1';
        v_io2  := '1';
        if ADDRESS(15 downto 13) = "100" and ((exrom_line = '0' and RWn = '1') or v_umax) then
            v_roml := '0';
        end if;
        if ADDRESS(15 downto 13) = "101" and exrom_line = '0' and game_line = '0' and RWn = '1' then
            v_romh := '0';
        end if;
        if ADDRESS(15 downto 13) = "111" and v_umax then
            v_romh := '0';
        end if;
        if ADDRESS(15 downto 8) = X"DE" then
            v_io1 := '0';
        end if;
        if ADDRESS(15 downto 8) = X"DF" then
            v_io2 := '0';
        end if;
        ROMLn <= transport v_roml after 30 ns;
        ROMHn <= transport v_romh after 30 ns;
        IO1n  <= transport v_io1  after 30 ns;
        IO2n  <= transport v_io2  after 30 ns;
    end process;

    bus_data <= cpu_wdata when cpu_drive else
                data_o    when data_t = '1' else
                (others => 'Z');

    ---------------------------------------------------------------------------
    -- DUT
    ---------------------------------------------------------------------------
    i_timing: entity work.slot_timing
    generic map (
        g_frequency    => c_freq )
    port map (
        clock          => clock,
        reset          => reset,
        PHI2           => PHI2,
        BA             => BA,
        serve_vic      => serve_vic,
        serve_enable   => serve_enable,
        serve_inhibit  => '0',
        timing_phi2    => X"B",
        timing_phi1    => X"B",
        edge_recover   => '0',
        allow_serve    => allow_serve,
        phi2_tick      => phi2_tick,
        phi2_fall      => open,
        phi2_recovered => open,
        dma_data_out   => open,
        clock_det      => open,
        vic_cycle      => open,
        prepare_dma    => open,
        refr_inhibit   => open,
        reqs_inhibit   => open,
        clear_inhibit  => clear_inhibit,
        do_sample_addr => do_sample_addr,
        do_sample_io   => do_sample_io,
        do_io_event    => do_io_event );

    i_slave: entity work.slot_slave
    port map (
        clock          => clock,
        reset          => reset,
        VCCDET         => '1',
        RSTn           => '1',
        PHI2           => PHI2,
        IO1n           => IO1n,
        IO2n           => IO2n,
        ROMLn          => ROMLn,
        ROMHn          => ROMHn,
        BA             => BA,
        GAMEn          => GAMEn,
        EXROMn         => EXROMn,
        RWn            => RWn,
        ADDRESS        => ADDRESS,
        DATA_in        => bus_data,
        DATA_out       => data_o,
        DATA_tri       => data_t,
        mem_req        => mem_req,
        mem_rwn        => mem_rwn,
        mem_rack       => mem_rack,
        mem_dack       => mem_dack,
        mem_rdata      => mem_rdata,
        mem_wdata      => mem_wdata,
        reset_out      => open,
        phi2_tick      => phi2_tick,
        do_sample_addr => do_sample_addr,
        do_probe_end   => '0',
        do_sample_io   => do_sample_io,
        do_io_event    => do_io_event,
        allow_serve    => allow_serve,
        serve_128      => serve_128,
        serve_rom      => serve_rom,
        serve_io1      => serve_io1,
        serve_io2      => serve_io2,
        serve_ultimax  => serve_ultimax,
        allow_write    => allow_write,
        kernal_enable  => '0',
        kernal_area    => kernal_area,
        clear_inhibit  => clear_inhibit,
        epyx_timeout   => epyx_timeout,
        cpu_write      => open,
        slot_req       => slot_req,
        slot_resp      => slot_resp,
        BUFFER_ENn     => open );

    i_cart: entity work.all_carts_v5
    generic map (
        g_eeprom        => false,
        g_ide64         => true,
        g_rom_base      => c_rom_base,
        g_ram_base      => c_ram_base )
    port map (
        clock           => clock,
        reset           => reset,
        io_req_eeprom   => c_io_req_init,
        io_resp_eeprom  => open,
        RST_in          => '0',
        c64_reset       => c64_reset,
        kernal_enable   => '0',
        kernal_area     => kernal_area,
        freeze_trig     => '0',
        freeze_act      => '0',
        freezer_ena     => open,
        unfreeze        => open,
        cart_active     => cart_active,
        cart_kill       => cart_kill,
        cart_logic      => cart_logic,
        cart_variant    => cart_variant,
        cart_force      => '0',
        slot_req        => slot_req,
        slot_resp       => slot_resp_cart,
        epyx_timeout    => epyx_timeout,
        serve_enable    => serve_enable,
        serve_vic       => serve_vic,
        serve_128       => serve_128,
        serve_rom       => serve_rom,
        serve_io1       => serve_io1,
        serve_io2       => serve_io2,
        serve_ultimax   => serve_ultimax,
        ide64_active    => ide64_active,
        ide64_rtc_ce    => ide64_rtc_ce,
        ide64_killed    => ide64_killed,
        allow_write     => allow_write,
        mem_req         => mem_req,
        mem_addr        => mem_addr,
        phi2            => PHI2,
        irq_n           => open,
        nmi_n           => open,
        exrom_n         => exrom_n,
        game_n          => game_n,
        CART_LEDn       => open );

    i_ata: entity work.ide64_ata
    port map (
        clock           => clock,
        reset           => reset,
        c64_reset       => c64_reset,
        active          => ide64_active,
        rtc_ce          => ide64_rtc_ce,
        killed          => ide64_killed,
        slot_req        => slot_req,
        slot_resp       => slot_resp_ide,
        io_req          => io_req,
        io_resp         => io_resp,
        io_irq          => io_irq );

    slot_resp <= or_reduce(slot_resp_cart & slot_resp_ide);

    ---------------------------------------------------------------------------
    -- Firmware model: serves the ATA device through its io_bus, the way the
    -- real driver does (event -> buffer -> status -> control).
    ---------------------------------------------------------------------------
    p_fw: process
        variable v_ev   : std_logic_vector(7 downto 0);
        variable v_cmd  : std_logic_vector(7 downto 0);
        variable v_n    : natural;
        variable v_lba  : natural;
        variable v_d    : std_logic_vector(7 downto 0);
        variable v_abort: boolean;

        procedure io_wr(a : natural; d : std_logic_vector(7 downto 0)) is
        begin
            io_req.address <= to_unsigned(a, 24);
            io_req.data    <= d;
            io_req.write   <= '1';
            wait until rising_edge(clock);
            io_req.write   <= '0';
            loop
                wait until rising_edge(clock);
                exit when io_resp.ack = '1';
            end loop;
        end procedure;

        procedure io_rd(a : natural; d : out std_logic_vector(7 downto 0)) is
        begin
            io_req.address <= to_unsigned(a, 24);
            io_req.read    <= '1';
            wait until rising_edge(clock);
            io_req.read    <= '0';
            loop
                wait until rising_edge(clock);
                exit when io_resp.ack = '1';
            end loop;
            d := io_resp.data;
        end procedure;

        procedure fill(lba : natural) is
        begin
            for i in 0 to 511 loop
                io_wr(16#200# + i, sec_byte(lba, i));
            end loop;
        end procedure;

        procedure wait_event(ev : out std_logic_vector(7 downto 0)) is
        begin
            if io_irq /= '1' and usb_cmd = usb_done then
                wait until io_irq = '1' or usb_cmd /= usb_done;
            end if;
            io_rd(to_integer(c_ide_events), ev);
        end procedure;

        procedure step_delay is
        begin
            if fw_slow then
                wait for 30 us;
            end if;
        end procedure;
    begin
        wait until reset = '0';
        wait for 100 ns;
        io_wr(to_integer(c_ide_present), X"81"); -- interface 0 master, IRQ on
        io_wr(16#100#, X"45"); -- DS1302 clock: 12:30:45
        io_wr(16#101#, X"30");
        io_wr(16#102#, X"12");
        loop
            wait_event(v_ev);
            if usb_cmd /= usb_done then
                case usb_kind is
                when 1 =>
                    io_wr(16#144#, X"03"); -- connected, flush
                    io_wr(16#400#, X"41");
                    io_wr(16#401#, X"42");
                    io_wr(16#402#, X"43");
                    io_wr(16#140#, X"03"); -- RX head
                when 2 =>
                    io_rd(16#142#, v_d); -- TX head
                    v_n := to_integer(unsigned(v_d));
                    usb_txn <= v_n;
                    io_rd(16#500#, v_d);
                    usb_tx0 <= v_d;
                    io_rd(16#502#, v_d);
                    usb_tx2 <= v_d;
                    io_wr(16#143#, std_logic_vector(to_unsigned(v_n, 8))); -- TX tail
                when others =>
                    io_wr(16#144#, X"02"); -- disconnected, flush
                end case;
                usb_done <= usb_cmd;
            end if;
            if v_ev(c_ide_ev_rtc_time) = '1' or v_ev(c_ide_ev_rtc_ram) = '1' then
                io_wr(to_integer(c_ide_events), v_ev and X"18");
                if v_ev(c_ide_ev_rtc_time) = '1' then
                    io_rd(16#100#, v_d);
                    fw_sec <= v_d;
                    fw_rtc_time <= fw_rtc_time + 1;
                end if;
                if v_ev(c_ide_ev_rtc_ram) = '1' then
                    io_rd(16#120#, v_d);
                    fw_ram0 <= v_d;
                    fw_rtc_ram <= fw_rtc_ram + 1;
                end if;
            elsif v_ev(c_ide_ev_reset) = '1' then
                io_wr(to_integer(c_ide_events), X"04");
                fw_resets <= fw_resets + 1;
            elsif v_ev(c_ide_ev_cmd) = '1' then
                io_wr(to_integer(c_ide_events), X"01");
                io_rd(to_integer(c_ide_command), v_cmd);
                case v_cmd is
                when X"EC" => -- IDENTIFY DEVICE
                    fill(1000);
                    step_delay;
                    io_wr(to_integer(c_ide_status), X"40");
                    io_wr(to_integer(c_ide_control), X"01");

                when X"20" => -- READ SECTORS (LBA low byte only)
                    io_rd(to_integer(c_ide_nsect), v_d);
                    v_n := to_integer(unsigned(v_d));
                    io_rd(to_integer(c_ide_sector), v_d);
                    v_lba := to_integer(unsigned(v_d));
                    v_abort := false;
                    for k in 0 to v_n - 1 loop
                        fill(v_lba + k);
                        step_delay;
                        io_wr(to_integer(c_ide_status), X"40");
                        if k < v_n - 1 then
                            io_wr(to_integer(c_ide_control), X"05"); -- more sectors follow
                            wait_event(v_ev);
                            if v_ev(c_ide_ev_reset) = '1' then
                                io_wr(to_integer(c_ide_events), X"04");
                                v_abort := true;
                            else
                                io_wr(to_integer(c_ide_events), X"02");
                            end if;
                        else
                            io_wr(to_integer(c_ide_control), X"01");
                        end if;
                        exit when v_abort;
                    end loop;

                when X"30" => -- WRITE SECTORS (one sector)
                    io_wr(to_integer(c_ide_status), X"40");
                    io_wr(to_integer(c_ide_control), X"03");
                    wait_event(v_ev);
                    if v_ev(c_ide_ev_reset) = '1' then
                        io_wr(to_integer(c_ide_events), X"04");
                    else
                        io_wr(to_integer(c_ide_events), X"02");
                        for i in 0 to 511 loop
                            io_rd(16#200# + i, v_d);
                            io_rd(16#200# + i, v_d);
                            wbuf(i) := v_d;
                        end loop;
                        fw_writes <= fw_writes + 1;
                        io_wr(to_integer(c_ide_status), X"40");
                        io_wr(to_integer(c_ide_control), X"00");
                    end if;

                when others => -- abort
                    io_wr(to_integer(c_ide_error), X"04");
                    io_wr(to_integer(c_ide_status), X"41");
                    io_wr(to_integer(c_ide_control), X"00");
                end case;
            end if;
        end loop;
    end process;

    ---------------------------------------------------------------------------
    -- Memory model: ROM at c_rom_base (rom_byte), 64K RAM at c_ram_base
    ---------------------------------------------------------------------------
    p_mem: process(clock)
        variable v_addr  : unsigned(25 downto 0);
        variable v_delay : integer := -1;
        variable v_data  : std_logic_vector(31 downto 0);

        impure function read_byte(a : unsigned(25 downto 0)) return std_logic_vector is
        begin
            if a(25 downto 20) = unsigned(c_rom_base(25 downto 20)) then
                return rom_byte(to_integer(a(19 downto 0)));
            elsif a(25 downto 16) = unsigned(c_ram_base(25 downto 16)) then
                return ram(to_integer(a(15 downto 0)));
            end if;
            return X"FF";
        end function;
    begin
        if rising_edge(clock) then
            mem_rack <= '0';
            mem_dack <= '0';
            if v_delay = 0 then
                mem_rdata <= v_data;
                mem_dack  <= '1';
            end if;
            if v_delay >= 0 then
                v_delay := v_delay - 1;
            end if;
            if mem_req = '1' and mem_rack = '0' then
                mem_rack <= '1';
                v_addr := mem_addr;
                if mem_rwn = '1' then
                    v_data := X"0000" & read_byte(v_addr + 1) & read_byte(v_addr);
                    v_delay := 4;
                else
                    if v_addr(25 downto 16) = unsigned(c_ram_base(25 downto 16)) then
                        ram(to_integer(v_addr(15 downto 0))) := mem_wdata(7 downto 0);
                    else
                        report "Unexpected memory write to " & hex(std_logic_vector("00" & v_addr)) severity error;
                    end if;
                end if;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Stimulus
    ---------------------------------------------------------------------------
    p_test: process
        variable v_err  : natural := 0;

        -- one CPU cycle (PHI2 high); returns what was on the bus at the end of PHI2
        procedure cycle(a : unsigned(15 downto 0); rw : std_logic; wd : std_logic_vector(7 downto 0);
                        rd : out std_logic_vector(7 downto 0); driven : out boolean) is
            variable t0 : time;
        begin
            wait until PHI2 = '1';
            t0 := now;
            wait for 10 ns;
            ADDRESS <= a;
            RWn     <= rw;
            if rw = '0' then
                wait for 90 ns;
                cpu_wdata <= wd;
                cpu_drive <= true;
            end if;
            -- sample just before the end of PHI2 (a write must not run into the next cycle)
            wait for (c_half_period - 30 ns) - (now - t0);
            rd     := bus_data;
            driven := data_t = '1';
            wait until PHI2 = '0';
            wait for 10 ns;
            cpu_drive <= false;
            ADDRESS   <= X"3FFF"; -- VIC fetch in PHI1
            RWn       <= '1';
        end procedure;

        procedure wr(a : unsigned(15 downto 0); d : std_logic_vector(7 downto 0)) is
            variable rd : std_logic_vector(7 downto 0);
            variable dr : boolean;
        begin
            cycle(a, '0', d, rd, dr);
        end procedure;

        procedure check(a : unsigned(15 downto 0); exp : std_logic_vector(7 downto 0); msg : string) is
            variable rd : std_logic_vector(7 downto 0);
            variable dr : boolean;
        begin
            cycle(a, '1', X"00", rd, dr);
            if not dr then
                report "FAIL " & msg & ": $" & hex(std_logic_vector(a)) & " not driven, expected $" & hex(exp) severity error;
                v_err := v_err + 1;
            elsif rd /= exp then
                report "FAIL " & msg & ": $" & hex(std_logic_vector(a)) & " = $" & hex(rd) & ", expected $" & hex(exp) severity error;
                v_err := v_err + 1;
            else
                report "ok   " & msg & ": $" & hex(std_logic_vector(a)) & " = $" & hex(rd);
            end if;
        end procedure;

        procedure check_open(a : unsigned(15 downto 0); msg : string) is
            variable rd : std_logic_vector(7 downto 0);
            variable dr : boolean;
        begin
            cycle(a, '1', X"00", rd, dr);
            if dr then
                report "FAIL " & msg & ": $" & hex(std_logic_vector(a)) & " driven with $" & hex(rd) & ", expected open bus" severity error;
                v_err := v_err + 1;
            else
                report "ok   " & msg & ": $" & hex(std_logic_vector(a)) & " open";
            end if;
        end procedure;

        procedure check_lines(game_phi2, game_phi1, exrom : std_logic; msg : string) is
            variable g2 : std_logic;
        begin
            wait until PHI2 = '1';
            wait for 200 ns;
            g2 := game_line;
            wait until PHI2 = '0';
            wait for 200 ns;
            if g2 /= game_phi2 or game_line /= game_phi1 or exrom_line /= exrom then
                report "FAIL " & msg & ": GAME phi2/phi1 = " & std_logic'image(g2) & "/" & std_logic'image(game_line) &
                       " EXROM = " & std_logic'image(exrom_line) severity error;
                v_err := v_err + 1;
            else
                report "ok   " & msg & ": GAME phi2/phi1 = " & std_logic'image(g2) & "/" & std_logic'image(game_line) &
                       " EXROM = " & std_logic'image(exrom_line);
            end if;
        end procedure;

        procedure c64_reset_pulse is
        begin
            wait until rising_edge(clock);
            c64_reset <= '1';
            wait for 1 us;
            wait until rising_edge(clock);
            c64_reset <= '0';
        end procedure;

        procedure rd_reg(a : unsigned(15 downto 0); d : out std_logic_vector(7 downto 0)) is
            variable dr : boolean;
        begin
            cycle(a, '1', X"00", d, dr);
        end procedure;

        -- poll $DE27 until it reads exp (up to 200 reads)
        procedure wait_status(exp : std_logic_vector(7 downto 0); msg : string) is
            variable d : std_logic_vector(7 downto 0);
        begin
            for i in 1 to 200 loop
                rd_reg(X"DE27", d);
                if d = exp then
                    report "ok   " & msg & ": status $" & hex(d) & " after " & integer'image(i) & " polls";
                    return;
                end if;
            end loop;
            report "FAIL " & msg & ": status $" & hex(d) & ", expected $" & hex(exp) severity error;
            v_err := v_err + 1;
        end procedure;

        -- read 256 words through $DE20/$DE31 and compare with sec_byte(lba)
        procedure read_sector(lba : natural; msg : string) is
            variable lo, hi : std_logic_vector(7 downto 0);
            variable bad    : natural := 0;
        begin
            for i in 0 to 255 loop
                rd_reg(X"DE20", lo);
                rd_reg(X"DE31", hi);
                if lo /= sec_byte(lba, 2 * i) or hi /= sec_byte(lba, 2 * i + 1) then
                    if bad < 4 then
                        report "FAIL " & msg & ": word " & integer'image(i) & " = $" & hex(hi) & hex(lo) &
                               ", expected $" & hex(sec_byte(lba, 2 * i + 1)) & hex(sec_byte(lba, 2 * i)) severity error;
                    end if;
                    bad := bad + 1;
                end if;
            end loop;
            if bad = 0 then
                report "ok   " & msg & ": 256 words";
            else
                v_err := v_err + 1;
            end if;
        end procedure;

        variable v_n : natural;
        variable v_b : std_logic_vector(7 downto 0);
        constant c_b3 : natural := 3 * 16384;

        -- DS1302: shift a byte in (LSB first), one $DE5F write per bit
        procedure rtc_out(v : std_logic_vector(7 downto 0)) is
        begin
            for i in 0 to 7 loop
                wr(X"DE5F", "0000000" & v(i));
            end loop;
        end procedure;

        -- DS1302: shift a byte out, one $DE5F read per bit
        procedure rtc_in(v : out std_logic_vector(7 downto 0)) is
            variable d : std_logic_vector(7 downto 0);
        begin
            for i in 0 to 7 loop
                rd_reg(X"DE5F", d);
                v(i) := d(0);
            end loop;
        end procedure;

        procedure rtc_check(cmd, exp : std_logic_vector(7 downto 0); msg : string) is
            variable v : std_logic_vector(7 downto 0);
        begin
            wr(X"DEFB", X"02"); -- CE high
            rtc_out(cmd);
            rtc_in(v);
            wr(X"DEFB", X"00"); -- CE low
            if v /= exp then
                report "FAIL " & msg & ": $" & hex(v) & ", expected $" & hex(exp) severity error;
                v_err := v_err + 1;
            else
                report "ok   " & msg & ": $" & hex(v);
            end if;
        end procedure;

        procedure rtc_write(cmd, v : std_logic_vector(7 downto 0)) is
        begin
            wr(X"DEFB", X"02");
            rtc_out(cmd);
            rtc_out(v);
            wr(X"DEFB", X"00");
        end procedure;

        procedure usb_fw(kind : natural) is
            variable n : natural;
        begin
            n := usb_cmd + 1;
            usb_kind <= kind;
            usb_cmd  <= n;
            wait until usb_done = n;
        end procedure;

        procedure check_count(actual, exp : natural; msg : string) is
        begin
            if actual /= exp then
                report "FAIL " & msg & ": " & integer'image(actual) & ", expected " & integer'image(exp) severity error;
                v_err := v_err + 1;
            else
                report "ok   " & msg & ": " & integer'image(actual);
            end if;
        end procedure;
    begin
        reset <= '1';
        wait for 200 ns;
        reset <= '0';
        c64_reset_pulse;
        for i in 1 to 4 loop
            wait until PHI2 = '1';
        end loop;

        ------------------------------------------------------------------ V4.1
        report "--- V4.1, after reset: 8K mode, bank 0";
        check_lines('1', '1', '0', "8K lines");
        check(X"DE32", X"22", "status 8K bank 0");
        check(X"8000", rom_byte(16#0000#), "ROML");
        check(X"9FFF", rom_byte(16#1FFF#), "ROML end");
        check_open(X"A000", "A000 in 8K");
        check(X"DE60", rom_byte(16#1E60#), "ROM window");
        check(X"DEFF", rom_byte(16#1EFF#), "ROM window end");
        check_open(X"DE00", "DE00 unclaimed");
        check_open(X"DE40", "DE40 unclaimed");
        check(X"DE5E", X"C0", "FT245 status");
        check_open(X"DF00", "IO2");

        report "--- bank select";
        wr(X"DE63", X"00");
        check(X"DE32", X"2E", "status bank 3");
        check(X"9000", rom_byte(c_b3 + 16#1000#), "ROML bank 3");
        check(X"DE80", rom_byte(c_b3 + 16#1E80#), "ROM window bank 3");
        wr(X"DE6F", X"00"); -- V4.1: DE68-DE7F do nothing
        check(X"DE32", X"2E", "DE6F ignored on V4.1");

        report "--- IDE bus latch (absent device 1 floats)";
        wr(X"DE31", X"A5");
        wr(X"DE26", X"F0");
        check(X"DE27", X"F0", "IDE bus low");
        check(X"DE31", X"A5", "IDE bus high");


        ------------------------------------------------------------------ ATA
        report "--- ATA: drive 0 present (firmware model)";
        wr(X"DE26", X"E0"); -- device 0, LBA
        check(X"DE27", X"40", "status DRDY");
        check(X"DE2E", X"40", "alternate status");
        check(X"DE26", X"E0", "device/head");
        check(X"DE2F", X"FE", "drive address dev 0 head 0");

        report "--- ATA: IDENTIFY DEVICE";
        wr(X"DE27", X"EC");
        wait_status(X"48", "IDENTIFY DRQ");
        read_sector(1000, "IDENTIFY");
        check(X"DE27", X"40", "IDENTIFY done");

        report "--- ATA: absent device 1 floats";
        wr(X"DE26", X"F0");
        check(X"DE22", X"F0", "dev 1 reg 2 = bus");
        check(X"DE27", X"F0", "dev 1 status = bus");
        wr(X"DE26", X"E0");
        check(X"DE27", X"40", "dev 0 again");

        report "--- ATA: READ SECTORS x2";
        wr(X"DE22", X"02");
        wr(X"DE23", X"05");
        wr(X"DE24", X"00");
        wr(X"DE25", X"00");
        wr(X"DE27", X"20");
        wait_status(X"48", "READ DRQ 1");
        read_sector(5, "READ lba 5");
        check(X"DE27", X"C0", "BSY between sectors");
        check(X"DE22", X"C0", "reg 2 reads status while BSY");
        wr(X"DE23", X"77"); -- ignored while BSY
        wait_status(X"48", "READ DRQ 2");
        read_sector(6, "READ lba 6");
        check(X"DE27", X"40", "READ done");
        check(X"DE23", X"05", "task file write ignored while BSY");

        report "--- ATA: WRITE SECTORS";
        wr(X"DE22", X"01");
        wr(X"DE27", X"30");
        wait_status(X"48", "WRITE DRQ");
        for i in 0 to 255 loop
            wr(X"DE31", sec_byte(77, 2 * i + 1));
            wr(X"DE20", sec_byte(77, 2 * i));
        end loop;
        wait_status(X"40", "WRITE done");
        if fw_writes /= 1 then
            report "FAIL WRITE: firmware received " & integer'image(fw_writes) & " sectors" severity error;
            v_err := v_err + 1;
        else
            v_n := 0;
            for i in 0 to 511 loop
                if wbuf(i) /= sec_byte(77, i) then
                    if v_n < 4 then
                        report "FAIL WRITE: byte " & integer'image(i) & " = $" & hex(wbuf(i)) &
                               ", expected $" & hex(sec_byte(77, i)) severity error;
                    end if;
                    v_n := v_n + 1;
                end if;
            end loop;
            if v_n /= 0 then
                report "FAIL WRITE: " & integer'image(v_n) & " bytes differ" severity error;
                v_err := v_err + 1;
            else
                report "ok   WRITE: 512 bytes received";
            end if;
        end if;

        report "--- ATA: unknown command aborts";
        wr(X"DE27", X"E5");
        wait_status(X"41", "abort status");
        check(X"DE21", X"04", "error ABRT");

        report "--- ATA: SRST in the middle of a command";
        fw_slow <= true;
        wr(X"DE22", X"01");
        wr(X"DE23", X"09");
        wr(X"DE27", X"20");
        check(X"DE27", X"C1", "BSY after command");
        wr(X"DE2E", X"04");
        wr(X"DE2E", X"00");
        check(X"DE21", X"01", "signature error");
        check(X"DE22", X"01", "signature nsect");
        check(X"DE23", X"01", "signature sector");
        check(X"DE24", X"00", "signature lcyl");
        check(X"DE25", X"00", "signature hcyl");
        check(X"DE27", X"40", "status after SRST");
        wait for 60 us; -- the late completion of the old command must be dropped
        check(X"DE27", X"40", "status after late completion");
        check(X"DE22", X"01", "nsect after late completion");
        fw_slow <= false;

        report "--- ATA: interface 1 has no device";
        wr(X"DE29", X"00");
        check(X"DE27", X"00", "interface 1 status = bus");
        wr(X"DE28", X"00");
        check(X"DE27", X"40", "interface 0 again");

        report "--- ATA: IDENTIFY after SRST";
        wr(X"DE27", X"EC");
        wait_status(X"48", "IDENTIFY DRQ");
        read_sector(1000, "IDENTIFY again");

        report "--- FT245";
        check_open(X"DE5D", "FT245 no data");
        usb_fw(1);
        check(X"DE5E", X"00", "FT245 status: data, can send");
        check(X"DE5D", X"41", "FT245 RX 1");
        check(X"DE5D", X"42", "FT245 RX 2");
        check(X"DE5D", X"43", "FT245 RX 3");
        check(X"DE5E", X"40", "FT245 status: RX empty");
        check_open(X"DE5D", "FT245 RX empty");
        wr(X"DE5D", X"61");
        wr(X"DE5D", X"62");
        wr(X"DE5D", X"63");
        usb_fw(2);
        check_count(usb_txn, 3, "FT245 bytes sent");
        if usb_tx0 /= X"61" or usb_tx2 /= X"63" then
            report "FAIL FT245 TX data $" & hex(usb_tx0) & " .. $" & hex(usb_tx2) severity error;
            v_err := v_err + 1;
        end if;
        usb_fw(3);
        check(X"DE5E", X"C0", "FT245 status: no link");
        wr(X"DE5D", X"64"); -- dropped
        usb_fw(2);
        check_count(usb_txn, 0, "FT245 bytes sent without link");

        report "--- DS1302";
        rtc_check(X"81", X"45", "read seconds");
        rtc_check(X"85", X"12", "read hours");
        rtc_write(X"C0", X"A5"); -- RAM 0
        wait for 2 us;
        check_count(fw_rtc_ram, 1, "RAM write events");
        if fw_ram0 /= X"A5" then
            report "FAIL RAM 0 seen by the CPU: $" & hex(fw_ram0) severity error;
            v_err := v_err + 1;
        end if;
        rtc_check(X"C1", X"A5", "read RAM 0");
        rtc_write(X"C2", X"5A"); -- RAM 1
        -- burst RAM read: the first two bytes
        wr(X"DEFB", X"02");
        rtc_out(X"FF");
        rtc_in(v_b);
        if v_b /= X"A5" then
            report "FAIL burst RAM byte 0: $" & hex(v_b) severity error;
            v_err := v_err + 1;
        end if;
        rtc_in(v_b);
        if v_b /= X"5A" then
            report "FAIL burst RAM byte 1: $" & hex(v_b) severity error;
            v_err := v_err + 1;
        else
            report "ok   burst RAM read";
        end if;
        wr(X"DEFB", X"00");
        -- CE dropped in the middle of a command byte restarts the protocol
        wr(X"DEFB", X"02");
        rtc_out(X"0F"); -- only 4 bits matter: 1111...
        wr(X"DEFB", X"00");
        rtc_check(X"83", X"30", "read minutes after CE abort");
        -- write protect blocks clock writes
        rtc_write(X"8E", X"80");
        rtc_write(X"80", X"00");
        wait for 2 us;
        check_count(fw_rtc_time, 0, "clock events while write protected");
        rtc_check(X"81", X"45", "seconds unchanged while write protected");
        rtc_write(X"8E", X"00");
        rtc_write(X"80", X"17");
        wait for 2 us;
        check_count(fw_rtc_time, 1, "clock events");
        if fw_sec /= X"17" then
            report "FAIL seconds seen by the CPU: $" & hex(fw_sec) severity error;
            v_err := v_err + 1;
        end if;
        rtc_check(X"81", X"17", "read seconds after write");

        report "--- 16K mode";
        wr(X"DEFC", X"00");
        check_lines('0', '1', '0', "16K lines");
        check(X"DE32", X"2C", "status 16K");
        check(X"A010", rom_byte(c_b3 + 16#2010#), "ROMH");
        check(X"8010", rom_byte(c_b3 + 16#0010#), "ROML in 16K");

        report "--- ultimax mode";
        wr(X"DEFE", X"00");
        check_lines('0', '1', '1', "ultimax lines");
        check(X"DE32", X"2D", "status ultimax");
        wr(X"1234", X"5A");
        wr(X"7FFF", X"A5");
        wr(X"1000", X"3C");
        check(X"1234", X"5A", "RAM 1234");
        check(X"7FFF", X"A5", "RAM 7FFF");
        check(X"1000", X"3C", "RAM 1000");
        if ram(16#1234#) /= X"5A" or ram(16#7FFF#) /= X"A5" then
            report "FAIL RAM offsets in cart memory" severity error;
            v_err := v_err + 1;
        end if;
        check(X"A100", rom_byte(c_b3 + 16#2100#), "ultimax A000 (ROM)");
        check(X"E200", rom_byte(c_b3 + 16#2200#), "ultimax E000 (ROMH)");
        check(X"8200", rom_byte(c_b3 + 16#0200#), "ultimax ROML");
        check_open(X"0800", "ultimax 0800");
        check_open(X"C000", "ultimax C000");
        wr(X"A000", X"77"); -- ROM write: ignored (no memory write)

        report "--- off";
        wr(X"DEFF", X"00");
        check_lines('1', '1', '1', "off lines");
        check(X"DE32", X"2F", "status off");
        check_open(X"8000", "8000 off");
        check_open(X"1234", "1234 off");

        report "--- 8K, then kill";
        wr(X"DEFD", X"00");
        check_lines('1', '1', '0', "8K lines again");
        wr(X"DEFB", X"03");
        check_lines('1', '1', '1', "killed lines");
        check(X"DE32", X"2F", "status killed");
        check_open(X"DE60", "ROM window killed");
        wr(X"DEFD", X"00");
        check_lines('1', '1', '1', "config ignored when killed");
        wr(X"DE61", X"00");
        check(X"DE32", X"2F", "bank ignored when killed");

        report "--- C64 reset clears kill";
        c64_reset_pulse;
        check(X"DE32", X"22", "status after reset");
        check(X"DE60", rom_byte(16#1E60#), "ROM window after reset");

        report "--- ATA: C64 reset held for 50 us raises one event";
        v_n := fw_resets;
        wait until rising_edge(clock);
        c64_reset <= '1';
        wait for 50 us;
        wait until rising_edge(clock);
        c64_reset <= '0';
        wait for 5 us;
        if fw_resets - v_n /= 1 then
            report "FAIL C64 reset: firmware saw " & integer'image(fw_resets - v_n) & " reset events, expected 1" severity error;
            v_err := v_err + 1;
        else
            report "ok   C64 reset: firmware saw 1 reset event";
        end if;
        check(X"DE27", X"40", "status after C64 reset");

        ------------------------------------------------------------------ V4.2
        report "--- V4.2";
        cart_variant <= "010";
        c64_reset_pulse;
        check(X"DE32", X"82", "V4.2 status");
        wr(X"DE7F", X"00");
        check(X"DE32", X"FE", "V4.2 bank 31");
        check(X"8000", rom_byte(31 * 16384), "V4.2 ROML bank 31");

        ------------------------------------------------------------------
        errors <= v_err;
        if v_err = 0 then
            report "*** tb_ide64: ALL TESTS PASSED ***";
        else
            report "*** tb_ide64: " & integer'image(v_err) & " FAILURES ***" severity error;
        end if;
        stop <= true;
        wait;
    end process;
end architecture;
