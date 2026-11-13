--------------------------------------------------------------------------------
-- Entity: ide64_usb
--
-- Description: The FT245 USB FIFO of the IDE64 V4.x cartridge, used by PCLink.
--              $DE5D read:  next received byte (bus left open when there is none)
--              $DE5D write: byte to send (dropped when full or not connected)
--              $DE5E read:  bit 7 = cannot send (TX full or no link),
--                           bit 6 = nothing received (RX empty or no link)
--              Follows VICE src/c64/cart/ide64.c (ide64_ft245_*).
--
--              The CPU moves the bytes to and from the network through two
--              rings (head/tail pointers, as in acia6551):
--                RX (to the C64):   4 KB, so the C64 doesn't run dry in the
--                                   middle of a reply while the CPU is busy
--                                   elsewhere (a real FT245 is refilled by USB
--                                   much faster than the C64 reads)
--                TX (from the C64): 256 bytes
--              CPU side (byte offsets):
--                reg 0  RX head low  (R/W; a write commits head = hold & low)
--                reg 5  RX head high (R; W = hold, write it before reg 0)
--                reg 1  RX tail low  (R; latches the tail high byte for reg 6)
--                reg 6  RX tail high (R, as latched by reading reg 1)
--                reg 2  TX head (R)       the C64 writes at the head
--                reg 3  TX tail (R/W)     the CPU takes bytes to send from the tail
--                reg 4  control: bit 0 = PC connected (R/W), write bit 1 = flush both rings
--                tx_en  TX ring, 256 bytes
--                rx_en  RX ring, 4 KB
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.io_bus_pkg.all;
use work.slot_bus_pkg.all;

entity ide64_usb is
port (
    clock           : in  std_logic;
    reset           : in  std_logic;
    active          : in  std_logic; -- the IDE64 cartridge is selected and enabled

    -- C64 side
    slot_req        : in  t_slot_req;
    slot_resp       : out t_slot_resp;

    -- CPU side
    reg_en          : in  std_logic;
    tx_en           : in  std_logic;
    rx_en           : in  std_logic;
    cpu_we          : in  std_logic;
    cpu_addr        : in  unsigned(11 downto 0);
    cpu_wdata       : in  std_logic_vector(7 downto 0);
    cpu_rdata       : out std_logic_vector(7 downto 0);
    cpu_ack         : out std_logic );
end entity;

architecture gideon of ide64_usb is
    signal rx_head      : unsigned(11 downto 0) := (others => '0');
    signal rx_tail      : unsigned(11 downto 0) := (others => '0');
    signal rx_head_hold : std_logic_vector(3 downto 0) := "0000";
    signal rx_tail_snap : std_logic_vector(3 downto 0) := "0000";
    signal tx_head      : unsigned(7 downto 0) := (others => '0');
    signal tx_tail      : unsigned(7 downto 0) := (others => '0');
    signal connected    : std_logic := '0';

    signal rx_empty     : std_logic;
    signal tx_full      : std_logic;

    signal c64_rd_data  : std_logic;
    signal c64_wr_data  : std_logic;
    signal rx_rdata     : std_logic_vector(7 downto 0);

    signal reg_rdata    : std_logic_vector(7 downto 0) := X"00";
    signal tx_cpu_rdata : std_logic_vector(7 downto 0);
    signal rx_cpu_rdata : std_logic_vector(7 downto 0);
    signal reg_ack      : std_logic := '0';
    signal tx_ack       : std_logic := '0';
    signal rx_ack       : std_logic := '0';
begin
    rx_empty <= '1' when rx_head = rx_tail else '0';
    tx_full  <= '1' when tx_head + 1 = tx_tail else '0';

    process(slot_req, active, rx_empty, tx_full, connected, rx_rdata)
    begin
        slot_resp <= c_slot_resp_init;
        if active = '1' then
            if slot_req.bus_address(8 downto 0) = "001011101" and rx_empty = '0' then -- DE5D
                slot_resp.data <= rx_rdata;
                slot_resp.reg_output <= '1';
            elsif slot_req.bus_address(8 downto 0) = "001011110" then -- DE5E
                slot_resp.data <= (tx_full or not connected) & (rx_empty or not connected) & "000000";
                slot_resp.reg_output <= '1';
            end if;
        end if;
    end process;

    -- pop when the read has completed (a halted 6510 repeats it; see io_read_done)
    c64_rd_data <= active and slot_req.io_read_done and not rx_empty when slot_req.io_done_address(8 downto 0) = "001011101" else '0';
    c64_wr_data <= active and slot_req.io_write and connected and not tx_full when slot_req.io_address(8 downto 0) = "001011101" else '0';

    process(clock)
    begin
        if rising_edge(clock) then
            if c64_rd_data = '1' then
                rx_tail <= rx_tail + 1;
            end if;
            if c64_wr_data = '1' then
                tx_head <= tx_head + 1;
            end if;

            reg_ack <= '0';
            if reg_en = '1' then
                reg_ack <= '1';
                if cpu_we = '1' then
                    case to_integer(cpu_addr(2 downto 0)) is
                    when 0 => rx_head <= unsigned(rx_head_hold) & unsigned(cpu_wdata);
                    when 5 => rx_head_hold <= cpu_wdata(3 downto 0);
                    when 3 => tx_tail <= unsigned(cpu_wdata);
                    when 4 =>
                        connected <= cpu_wdata(0);
                        if cpu_wdata(1) = '1' then
                            rx_head <= (others => '0');
                            rx_tail <= (others => '0');
                            tx_head <= (others => '0');
                            tx_tail <= (others => '0');
                        end if;
                    when others => null;
                    end case;
                else
                    case to_integer(cpu_addr(2 downto 0)) is
                    when 0 => reg_rdata <= std_logic_vector(rx_head(7 downto 0));
                    when 5 => reg_rdata <= "0000" & std_logic_vector(rx_head(11 downto 8));
                    when 1 =>
                        reg_rdata    <= std_logic_vector(rx_tail(7 downto 0));
                        rx_tail_snap <= std_logic_vector(rx_tail(11 downto 8));
                    when 6 => reg_rdata <= "0000" & rx_tail_snap;
                    when 2 => reg_rdata <= std_logic_vector(tx_head);
                    when 3 => reg_rdata <= std_logic_vector(tx_tail);
                    when 4 => reg_rdata <= "0000000" & connected;
                    when others => reg_rdata <= X"00";
                    end case;
                end if;
            end if;
            tx_ack <= tx_en;
            rx_ack <= rx_en;

            if reset = '1' then
                rx_head   <= (others => '0');
                rx_tail   <= (others => '0');
                tx_head   <= (others => '0');
                tx_tail   <= (others => '0');
                connected <= '0';
            end if;
        end if;
    end process;

    cpu_ack   <= reg_ack or tx_ack or rx_ack;
    cpu_rdata <= tx_cpu_rdata when tx_ack = '1' else
                 rx_cpu_rdata when rx_ack = '1' else
                 reg_rdata;

    -- RX: the CPU writes (port A), the C64 reads at the tail (port B, always addressed)
    i_rx: entity work.dpram
    generic map (
        g_width_bits    => 8,
        g_depth_bits    => 12,
        g_read_first_a  => false,
        g_read_first_b  => false,
        g_storage       => "block" )
    port map (
        a_clock         => clock,
        a_address       => cpu_addr,
        a_rdata         => rx_cpu_rdata,
        a_wdata         => cpu_wdata,
        a_en            => rx_en,
        a_we            => cpu_we,

        b_clock         => clock,
        b_address       => rx_tail,
        b_rdata         => rx_rdata,
        b_en            => '1',
        b_we            => '0' );

    -- TX: the C64 writes at the head (port B), the CPU reads (port A)
    i_tx: entity work.dpram
    generic map (
        g_width_bits    => 8,
        g_depth_bits    => 8,
        g_read_first_a  => false,
        g_read_first_b  => false,
        g_storage       => "block" )
    port map (
        a_clock         => clock,
        a_address       => cpu_addr(7 downto 0),
        a_rdata         => tx_cpu_rdata,
        a_wdata         => cpu_wdata,
        a_en            => tx_en,
        a_we            => cpu_we,

        b_clock         => clock,
        b_address       => tx_head,
        b_rdata         => open,
        b_wdata         => slot_req.data,
        b_en            => '1',
        b_we            => c64_wr_data );

end architecture;
