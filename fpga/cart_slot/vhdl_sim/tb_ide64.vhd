-------------------------------------------------------------------------------
-- Title      : tb_ide64
-------------------------------------------------------------------------------
-- Description: Testbench for the IDE64 cart logic (c_ide64 in all_carts_v5)
--              together with slot_slave and slot_timing. A simple C64 bus
--              model (CPU cycles in PHI2, simplified PLA) and a memory model
--              holding the cart ROM and RAM. Checks modes, banking, the
--              $DExx registers, ultimax RAM serving and the kill port.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.io_bus_pkg.all;
use work.slot_bus_pkg.all;

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
    signal errors           : natural := 0;

    function rom_byte(i : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(((i mod 251) + 37 * (i / 16384)) mod 256, 8));
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
        slot_resp       => slot_resp,
        epyx_timeout    => epyx_timeout,
        serve_enable    => serve_enable,
        serve_vic       => serve_vic,
        serve_128       => serve_128,
        serve_rom       => serve_rom,
        serve_io1       => serve_io1,
        serve_io2       => serve_io2,
        serve_ultimax   => serve_ultimax,
        allow_write     => allow_write,
        mem_req         => mem_req,
        mem_addr        => mem_addr,
        phi2            => PHI2,
        irq_n           => open,
        nmi_n           => open,
        exrom_n         => exrom_n,
        game_n          => game_n,
        CART_LEDn       => open );

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
        begin
            wait until PHI2 = '1';
            wait for 10 ns;
            ADDRESS <= a;
            RWn     <= rw;
            if rw = '0' then
                wait for 90 ns;
                cpu_wdata <= wd;
                cpu_drive <= true;
            end if;
            wait until PHI2 = '0' for c_half_period - 30 ns;
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

        constant c_b3 : natural := 3 * 16384;
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

        report "--- IDE bus latch";
        wr(X"DE31", X"A5");
        wr(X"DE26", X"E0");
        check(X"DE27", X"E0", "IDE bus low");
        check(X"DE31", X"A5", "IDE bus high");

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
