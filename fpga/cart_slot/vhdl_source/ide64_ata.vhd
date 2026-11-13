--------------------------------------------------------------------------------
-- Entity: ide64_ata
--
-- Description: The IDE bus of the IDE64 V4.x cartridge ($DE20-$DE2F, $DE31)
--              with up to four emulated ATA devices (2 interfaces x master/slave)
--              behind one shared task file.
--
--              The hardware does everything that has to happen within a C64
--              cycle: the task file, the 16-bit data latches, BSY/DRQ, the
--              word pointer into a 512-byte sector buffer, and SRST. The CPU
--              (firmware) executes the commands: it gets an event + IRQ when
--              a command is written or a buffer has been drained or filled,
--              fills/empties the buffer and completes the step through the
--              control register. See ide64_ata_pkg for the CPU register map.
--
--              Behaviour follows VICE (src/core/ata.c, src/c64/cart/ide64.c):
--              - a word at $DE20 is low byte; reading $DE20 latches the high
--                byte for $DE31. Writing $DE31 sets the high byte of the next
--                $DE2x write.
--              - task file writes are ignored while BSY or DRQ; commands to an
--                absent device are ignored, except $90 (EXECUTE DIAGNOSTIC).
--              - while BSY, registers 1-6 read as status.
--              - an absent device leaves the bus floating: reads return the
--                last value on the IDE bus.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.io_bus_pkg.all;
use work.slot_bus_pkg.all;
use work.ide64_ata_pkg.all;

entity ide64_ata is
port (
    clock           : in  std_logic;
    reset           : in  std_logic;
    c64_reset       : in  std_logic := '0';
    active          : in  std_logic; -- the IDE64 cartridge is selected and enabled

    -- C64 side
    slot_req        : in  t_slot_req;
    slot_resp       : out t_slot_resp;

    -- CPU side
    io_req          : in  t_io_req;
    io_resp         : out t_io_resp;
    io_irq          : out std_logic );
end entity;

architecture gideon of ide64_ata is
    -- task file
    signal features     : std_logic_vector(7 downto 0) := X"00";
    signal error        : std_logic_vector(7 downto 0) := X"01";
    signal nsect        : std_logic_vector(7 downto 0) := X"01";
    signal sector       : std_logic_vector(7 downto 0) := X"01";
    signal lcyl         : std_logic_vector(7 downto 0) := X"00";
    signal hcyl         : std_logic_vector(7 downto 0) := X"00";
    signal selreg       : std_logic_vector(7 downto 0) := X"00"; -- device/head
    signal command      : std_logic_vector(7 downto 0) := X"00";
    signal devctl       : std_logic_vector(7 downto 0) := X"00";
    signal status_base  : std_logic_vector(7 downto 0) := X"40"; -- DRDY

    -- protocol state
    signal iface        : std_logic := '0';
    signal bsy          : std_logic := '0';
    signal xfer         : std_logic := '0'; -- data phase active
    signal dir_out      : std_logic := '0'; -- data phase direction: C64 -> buffer
    signal busy_at_end  : std_logic := '0';
    signal ptr          : unsigned(7 downto 0) := (others => '0'); -- word pointer

    -- C64 bus latches
    signal out_hi       : std_logic_vector(7 downto 0) := X"00"; -- $DE31 write
    signal in_hi        : std_logic_vector(7 downto 0) := X"00"; -- $DE31 read
    signal ide_bus      : std_logic_vector(15 downto 0) := X"0000"; -- last value on the IDE bus

    -- CPU interface
    signal present      : std_logic_vector(3 downto 0) := "0000";
    signal irq_en       : std_logic := '0';
    signal ev_cmd       : std_logic := '0';
    signal ev_buf       : std_logic := '0';
    signal ev_reset     : std_logic := '0';
    signal c64_reset_d  : std_logic := '0';

    -- derived
    signal bsy_eff      : std_logic;
    signal drq          : std_logic;
    signal status       : std_logic_vector(7 downto 0);
    signal sel_present  : std_logic;
    signal any_present  : std_logic;
    signal reg_rdata    : std_logic_vector(7 downto 0);
    signal sel_idx      : unsigned(1 downto 0);

    -- C64 access decode
    signal c64_rd_reg   : std_logic; -- io_read  of $DE20-$DE2F
    signal c64_wr_reg   : std_logic; -- io_write of $DE20-$DE2F
    signal c64_wr_hi    : std_logic; -- io_write of $DE31
    signal data_rd      : std_logic; -- data word read from the buffer
    signal data_wr      : std_logic; -- data word written into the buffer

    -- sector buffer: even bytes (low) and odd bytes (high)
    signal buf_en_lo    : std_logic;
    signal buf_en_hi    : std_logic;
    signal buf_a_lo     : std_logic_vector(7 downto 0);
    signal buf_a_hi     : std_logic_vector(7 downto 0);
    signal buf_b_lo     : std_logic_vector(7 downto 0);
    signal buf_b_hi     : std_logic_vector(7 downto 0);
    signal buf_ack      : std_logic := '0';
    signal wr_pend      : std_logic := '0'; -- C64 data word: written one clock after the event
    signal wr_addr      : unsigned(7 downto 0) := (others => '0');
    signal wr_lo        : std_logic_vector(7 downto 0) := X"00";
    signal wr_hi        : std_logic_vector(7 downto 0) := X"00";
    signal buf_b_addr   : unsigned(7 downto 0);
    signal buf_odd      : std_logic := '0';
    signal regs_resp    : t_io_resp := c_io_resp_init;
begin
    bsy_eff     <= bsy or devctl(2); -- SRST held = busy
    drq         <= xfer and not bsy_eff;
    status      <= bsy_eff & status_base(6 downto 4) & drq & status_base(2 downto 0);
    sel_idx     <= iface & selreg(4);
    sel_present <= present(to_integer(sel_idx));
    any_present <= present(1) or present(0) when iface='0' else present(3) or present(2);

    -- Read data of $DE20-$DE2F, from the live bus address
    process(slot_req, sel_present, bsy_eff, status, error, nsect, sector, lcyl, hcyl, selreg,
            xfer, dir_out, buf_b_lo, ide_bus)
        variable v_reg  : unsigned(3 downto 0);
        variable v_addr : std_logic_vector(6 downto 0);
    begin
        v_reg := slot_req.bus_address(3 downto 0);
        if bsy_eff='1' and v_reg >= 1 and v_reg <= 6 then
            v_reg := X"E"; -- read as status while busy
        end if;
        v_addr := "1111111";
        v_addr(5 downto 2) := v_addr(5 downto 2) xor selreg(3 downto 0);
        if selreg(4)='0' then
            v_addr(0) := '0';
        else
            v_addr(1) := '0';
        end if;

        reg_rdata <= ide_bus(7 downto 0);
        if sel_present='1' then
            case to_integer(v_reg) is
            when 0 =>
                if xfer='1' and dir_out='0' and bsy_eff='0' then
                    reg_rdata <= buf_b_lo;
                end if;
            when 1 =>  reg_rdata <= error;
            when 2 =>  reg_rdata <= nsect;
            when 3 =>  reg_rdata <= sector;
            when 4 =>  reg_rdata <= lcyl;
            when 5 =>  reg_rdata <= hcyl;
            when 6 =>  reg_rdata <= selreg;
            when 7 | 14 => reg_rdata <= status;
            when 15 => reg_rdata <= ide_bus(7) & v_addr;
            when others => null;
            end case;
        end if;
    end process;

    process(slot_req, active, reg_rdata, in_hi)
    begin
        slot_resp <= c_slot_resp_init;
        if active='1' then
            if slot_req.bus_address(8 downto 4) = "00010" then -- DE20-DE2F
                slot_resp.data <= reg_rdata;
                slot_resp.reg_output <= '1';
            elsif slot_req.bus_address(8 downto 0) = "000110001" then -- DE31
                slot_resp.data <= in_hi;
                slot_resp.reg_output <= '1';
            end if;
        end if;
    end process;

    c64_rd_reg <= active and slot_req.io_read  when slot_req.io_address(8 downto 4) = "00010" else '0';
    c64_wr_reg <= active and slot_req.io_write when slot_req.io_address(8 downto 4) = "00010" else '0';
    c64_wr_hi  <= active and slot_req.io_write when slot_req.io_address(8 downto 0) = "000110001" else '0';

    data_rd <= c64_rd_reg and sel_present and xfer and not dir_out and not bsy_eff when slot_req.io_address(3 downto 0) = X"0" else '0';
    data_wr <= c64_wr_reg and sel_present and xfer and dir_out and not bsy_eff when slot_req.io_address(3 downto 0) = X"0" else '0';

    io_irq <= irq_en and (ev_cmd or ev_buf or ev_reset);

    process(clock)
        variable v_reg  : natural range 0 to 15;
        variable v_srst : boolean;
        variable v_iface: std_logic;
    begin
        if rising_edge(clock) then
            v_srst := false;
            v_reg  := to_integer(slot_req.io_address(3 downto 0));

            ---------------------------------------------------------------
            -- C64 side
            ---------------------------------------------------------------
            if c64_wr_hi='1' then
                out_hi <= slot_req.data;
            end if;

            if c64_rd_reg='1' then
                if data_rd='1' then
                    in_hi   <= buf_b_hi;
                    ide_bus <= buf_b_hi & buf_b_lo;
                    if ptr = 255 then
                        ptr  <= (others => '0');
                        xfer <= '0';
                        if busy_at_end='1' then
                            bsy    <= '1';
                            ev_buf <= '1';
                        end if;
                    else
                        ptr <= ptr + 1;
                    end if;
                else
                    in_hi <= ide_bus(15 downto 8);
                end if;
            end if;

            if c64_wr_reg='1' then
                ide_bus <= out_hi & slot_req.data;
                v_iface := iface;
                if v_reg = 8 or v_reg = 9 then
                    v_iface := slot_req.io_address(0);
                    iface   <= v_iface;
                end if;

                case v_reg is
                when 0 =>
                    if data_wr='1' then
                        if ptr = 255 then
                            ptr    <= (others => '0');
                            xfer   <= '0';
                            bsy    <= '1';
                            ev_buf <= '1';
                        else
                            ptr <= ptr + 1;
                        end if;
                    end if;
                when 1 to 6 =>
                    if any_present='1' and bsy_eff='0' and xfer='0' then
                        case v_reg is
                        when 1 => features <= slot_req.data;
                        when 2 => nsect    <= slot_req.data;
                        when 3 => sector   <= slot_req.data;
                        when 4 => lcyl     <= slot_req.data;
                        when 5 => hcyl     <= slot_req.data;
                        when others => selreg <= slot_req.data;
                        end case;
                    end if;
                when 7 =>
                    if any_present='1' and bsy_eff='0' and xfer='0' and
                       (sel_present='1' or slot_req.data = X"90") then
                        command <= slot_req.data;
                        bsy     <= '1';
                        ev_cmd  <= '1';
                    end if;
                when 14 =>
                    devctl <= slot_req.data;
                    if devctl(2)='1' and slot_req.data(2)='0' then
                        v_srst := true;
                    end if;
                when others =>
                    null;
                end case;
            end if;

            ---------------------------------------------------------------
            -- CPU side
            ---------------------------------------------------------------
            regs_resp <= c_io_resp_init;
            if io_req.address(9)='0' then
                if io_req.write='1' then
                    regs_resp.ack <= '1';
                    if ev_reset='0' then -- completions are dropped until the reset is acknowledged
                        case io_req.address(3 downto 0) is
                        when c_ide_error   => error  <= io_req.data;
                        when c_ide_nsect   => nsect  <= io_req.data;
                        when c_ide_sector  => sector <= io_req.data;
                        when c_ide_lcyl    => lcyl   <= io_req.data;
                        when c_ide_hcyl    => hcyl   <= io_req.data;
                        when c_ide_select  => selreg <= io_req.data;
                        when c_ide_status  => status_base <= io_req.data and X"77";
                        when c_ide_control =>
                            bsy         <= '0';
                            xfer        <= io_req.data(0);
                            dir_out     <= io_req.data(1);
                            busy_at_end <= io_req.data(2);
                            ptr         <= (others => '0');
                        when others => null;
                        end case;
                    end if;
                    case io_req.address(3 downto 0) is
                    when c_ide_events =>
                        if io_req.data(c_ide_ev_cmd)='1' then
                            ev_cmd <= '0';
                        end if;
                        if io_req.data(c_ide_ev_buf)='1' then
                            ev_buf <= '0';
                        end if;
                        if io_req.data(c_ide_ev_reset)='1' then
                            ev_reset <= '0';
                        end if;
                    when c_ide_present =>
                        present <= io_req.data(3 downto 0);
                        irq_en  <= io_req.data(7);
                    when others => null;
                    end case;
                elsif io_req.read='1' then
                    regs_resp.ack <= '1';
                    case io_req.address(3 downto 0) is
                    when c_ide_features => regs_resp.data <= features;
                    when c_ide_error    => regs_resp.data <= error;
                    when c_ide_nsect    => regs_resp.data <= nsect;
                    when c_ide_sector   => regs_resp.data <= sector;
                    when c_ide_lcyl     => regs_resp.data <= lcyl;
                    when c_ide_hcyl     => regs_resp.data <= hcyl;
                    when c_ide_select   => regs_resp.data <= selreg;
                    when c_ide_command  => regs_resp.data <= command;
                    when c_ide_status   => regs_resp.data <= status;
                    when c_ide_control  => regs_resp.data <= bsy & "0000" & busy_at_end & dir_out & xfer;
                    when c_ide_events   => regs_resp.data <= "00000" & ev_reset & ev_buf & ev_cmd;
                    when c_ide_present  => regs_resp.data <= irq_en & "000" & present;
                    when c_ide_devctl   => regs_resp.data <= devctl;
                    when c_ide_iface    => regs_resp.data <= active & "000000" & iface;
                    when c_ide_pointer  => regs_resp.data <= std_logic_vector(ptr);
                    when others         => null;
                    end case;
                end if;
            end if;

            ---------------------------------------------------------------
            -- Resets: SRST (1 -> 0 edge of device control bit 2), C64 reset.
            -- The state is held in reset while the C64 reset is active; the
            -- event is raised once, when it is released (a level event would
            -- interrupt the CPU continuously while the C64 is held in reset).
            ---------------------------------------------------------------
            c64_reset_d <= c64_reset;
            if v_srst or c64_reset='1' or reset='1' then
                -- drive_diag() signature
                error       <= X"01";
                nsect       <= X"01";
                sector      <= X"01";
                lcyl        <= X"00";
                hcyl        <= X"00";
                selreg      <= X"00";
                status_base <= X"40";
                bsy         <= '0';
                xfer        <= '0';
                busy_at_end <= '0';
                ptr         <= (others => '0');
                ev_cmd      <= '0';
                ev_buf      <= '0';
            end if;
            if v_srst or (c64_reset_d='1' and c64_reset='0') then
                ev_reset    <= '1';
            end if;
            if c64_reset='1' or reset='1' then
                iface  <= '0';
                devctl <= X"00";
            end if;
            if reset='1' then
                present  <= "0000";
                irq_en   <= '0';
                ev_reset <= '0';
                out_hi   <= X"00";
                in_hi    <= X"00";
                ide_bus  <= X"0000";
            end if;
        end if;
    end process;

    -- CPU access to the sector buffer: offsets $200-$3FF, address bit 0 selects the RAM
    buf_en_lo <= (io_req.read or io_req.write) and io_req.address(9) and not io_req.address(0);
    buf_en_hi <= (io_req.read or io_req.write) and io_req.address(9) and io_req.address(0);

    process(clock)
    begin
        if rising_edge(clock) then
            buf_ack <= (io_req.read or io_req.write) and io_req.address(9);
            buf_odd <= io_req.address(0);

            wr_pend <= data_wr;
            if data_wr='1' then
                wr_addr <= ptr;
                wr_lo   <= slot_req.data;
                wr_hi   <= out_hi;
            end if;
        end if;
    end process;

    -- Port B reads the word at the pointer, except for the clock that writes a C64 data word
    buf_b_addr <= wr_addr when wr_pend='1' else ptr;

    io_resp.ack  <= regs_resp.ack or buf_ack;
    io_resp.data <= regs_resp.data when buf_ack='0' else
                    buf_a_hi when buf_odd='1' else
                    buf_a_lo;

    i_buf_lo: entity work.dpram
    generic map (
        g_width_bits    => 8,
        g_depth_bits    => 8,
        g_read_first_a  => false,
        g_read_first_b  => false,
        g_storage       => "block" )
    port map (
        a_clock         => clock,
        a_address       => io_req.address(8 downto 1),
        a_rdata         => buf_a_lo,
        a_wdata         => io_req.data,
        a_en            => buf_en_lo,
        a_we            => io_req.write,

        b_clock         => clock,
        b_address       => buf_b_addr,
        b_rdata         => buf_b_lo,
        b_wdata         => wr_lo,
        b_en            => '1',
        b_we            => wr_pend );

    i_buf_hi: entity work.dpram
    generic map (
        g_width_bits    => 8,
        g_depth_bits    => 8,
        g_read_first_a  => false,
        g_read_first_b  => false,
        g_storage       => "block" )
    port map (
        a_clock         => clock,
        a_address       => io_req.address(8 downto 1),
        a_rdata         => buf_a_hi,
        a_wdata         => io_req.data,
        a_en            => buf_en_hi,
        a_we            => io_req.write,

        b_clock         => clock,
        b_address       => buf_b_addr,
        b_rdata         => buf_b_hi,
        b_wdata         => wr_hi,
        b_en            => '1',
        b_we            => wr_pend );

end architecture;
