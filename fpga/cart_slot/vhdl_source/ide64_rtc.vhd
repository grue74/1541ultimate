--------------------------------------------------------------------------------
-- Entity: ide64_rtc
--
-- Description: The DS1302 real time clock of the IDE64 cartridge, at $DE5F.
--              CE comes from $DEFB bit 1; every $DE5F access is one SCLK
--              pulse: falling edge (an output bit is shifted out), then rising
--              edge (an input bit is shifted in: bit 0 of the written value,
--              or 1 on a read). Follows VICE src/core/rtc/ds1202_1302.c.
--
--              The serial engine is here; the registers live in a 64-byte
--              register file that the CPU shares:
--                0-7   clock (seconds/CH, minutes, hours, date, month, day,
--                      year, write protect), 8 trickle charge
--                32-63 the 31 (+1) bytes of RAM
--              The CPU keeps the clock registers up to date and is told when
--              the C64 writes a clock register or the RAM.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ide64_rtc is
port (
    clock           : in  std_logic;
    reset           : in  std_logic;

    -- C64 side
    ce              : in  std_logic;
    access_strobe   : in  std_logic; -- one $DE5F access, at the end of the cycle
    access_write    : in  std_logic;
    access_wbit     : in  std_logic; -- bit 0 of the written value
    rbit            : out std_logic; -- the bit a $DE5F read returns

    -- CPU side
    cpu_en          : in  std_logic;
    cpu_we          : in  std_logic;
    cpu_addr        : in  unsigned(5 downto 0);
    cpu_wdata       : in  std_logic_vector(7 downto 0);
    cpu_rdata       : out std_logic_vector(7 downto 0);

    ev_time         : out std_logic; -- pulse: the C64 wrote clock register 0-6
    ev_ram          : out std_logic ); -- pulse: the C64 wrote the RAM
end entity;

architecture gideon of ide64_rtc is
    type t_state is (cmd, in_single, in_burst, out_single, out_burst);
    signal state        : t_state := cmd;
    signal bitcnt       : unsigned(2 downto 0) := "000";
    signal io_byte      : std_logic_vector(7 downto 0) := X"00";
    signal reg          : unsigned(5 downto 0) := (others => '0'); -- register file address
    signal wp           : std_logic := '0'; -- write protect (register 7, bit 7)

    signal b_addr       : unsigned(5 downto 0) := (others => '0');
    signal b_wdata      : std_logic_vector(7 downto 0) := X"00";
    signal b_we         : std_logic := '0';
    signal b_rdata      : std_logic_vector(7 downto 0);
    signal load         : std_logic_vector(1 downto 0) := "00"; -- io_byte <= register file, in 2 clocks
begin
    rbit <= io_byte(to_integer(bitcnt)) when state = out_single or state = out_burst else '1';

    process(clock)
        variable v_byte : std_logic_vector(7 downto 0);
        variable v_reg  : unsigned(5 downto 0);
    begin
        if rising_edge(clock) then
            b_we    <= '0';
            ev_time <= '0';
            ev_ram  <= '0';

            load <= '0' & load(1);
            if load(0) = '1' then
                io_byte <= b_rdata;
            end if;

            if access_strobe = '1' and ce = '1' then
                case state is
                when out_single | out_burst =>
                    -- falling edge: this bit has been read; rising edge: no input
                    bitcnt <= bitcnt + 1;
                    if bitcnt = 7 then
                        if state = out_single then
                            state <= cmd;
                            io_byte <= X"00";
                        else
                            v_reg := reg + 1;
                            reg <= v_reg;
                            if (reg(5) = '0' and reg(4 downto 0) = 7) or reg = 63 then -- 8 clock or 32 RAM bytes
                                state <= cmd;
                                io_byte <= X"00";
                            else
                                b_addr <= v_reg;
                                load   <= "10";
                            end if;
                        end if;
                    end if;

                when others =>
                    -- rising edge: shift in one bit, LSB first
                    v_byte := io_byte;
                    if access_write = '1' then
                        v_byte(to_integer(bitcnt)) := access_wbit;
                    else
                        v_byte(to_integer(bitcnt)) := '1';
                    end if;
                    io_byte <= v_byte;
                    bitcnt  <= bitcnt + 1;

                    if bitcnt = 7 then
                        io_byte <= X"00";
                        case state is
                        when cmd =>
                            if v_byte(7) = '0' then
                                null; -- not a command: stay in command mode
                            else
                                -- bit 6: 1 = RAM, 0 = clock; bits 5-1: register (31 = burst); bit 0: read
                                v_reg(5) := v_byte(6);
                                v_reg(4 downto 0) := unsigned(v_byte(5 downto 1));
                                if v_byte(5 downto 1) = "11111" then -- burst starts at register 0
                                    v_reg(4 downto 0) := "00000";
                                end if;
                                reg <= v_reg;
                                if v_byte(0) = '1' then
                                    if v_byte(5 downto 1) = "11111" then
                                        state <= out_burst;
                                    else
                                        state <= out_single;
                                    end if;
                                    b_addr <= v_reg;
                                    load   <= "10";
                                else
                                    if v_byte(5 downto 1) = "11111" then
                                        state <= in_burst;
                                    else
                                        state <= in_single;
                                    end if;
                                end if;
                            end if;

                        when in_single | in_burst =>
                            -- write the byte; clock registers 0-6 only when not write protected
                            if reg(5) = '1' then
                                b_addr  <= reg;
                                b_wdata <= v_byte;
                                b_we    <= '1';
                                ev_ram  <= '1';
                            elsif reg(4 downto 0) = 7 then
                                b_addr  <= reg;
                                b_wdata <= v_byte;
                                b_we    <= '1';
                                wp      <= v_byte(7);
                            elsif reg(4 downto 0) = 8 then
                                b_addr  <= reg;
                                b_wdata <= v_byte;
                                b_we    <= '1';
                            elsif reg(4 downto 0) < 7 and wp = '0' then
                                b_addr  <= reg;
                                b_wdata <= v_byte;
                                b_we    <= '1';
                                ev_time <= '1';
                            end if;

                            if state = in_single then
                                state <= cmd;
                            else
                                reg <= reg + 1;
                                if (reg(5) = '0' and reg(4 downto 0) = 7) or reg = 63 then
                                    state <= cmd;
                                end if;
                            end if;

                        when others =>
                            null;
                        end case;
                    end if;
                end case;
            end if;

            -- the CPU sets write protect directly too
            if cpu_en = '1' and cpu_we = '1' and cpu_addr = 7 then
                wp <= cpu_wdata(7);
            end if;

            if ce = '0' or reset = '1' then
                state   <= cmd;
                bitcnt  <= "000";
                if load = "00" then
                    io_byte <= X"00";
                end if;
            end if;
            if reset = '1' then
                wp <= '0';
            end if;
        end if;
    end process;

    i_regs: entity work.dpram
    generic map (
        g_width_bits    => 8,
        g_depth_bits    => 6,
        g_read_first_a  => false,
        g_read_first_b  => false,
        g_storage       => "block" )
    port map (
        a_clock         => clock,
        a_address       => cpu_addr,
        a_rdata         => cpu_rdata,
        a_wdata         => cpu_wdata,
        a_en            => cpu_en,
        a_we            => cpu_we,

        b_clock         => clock,
        b_address       => b_addr,
        b_rdata         => b_rdata,
        b_wdata         => b_wdata,
        b_en            => '1',
        b_we            => b_we );

end architecture;
