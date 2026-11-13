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
--              Two 256-byte rings in one block RAM; the CPU moves the bytes to
--              and from the network (head/tail pointers, as in acia6551).
--              CPU side (byte offsets):
--                reg 0  RX head (R/W)   the CPU writes received bytes at the head
--                reg 1  RX tail (R)     the C64 reads at the tail
--                reg 2  TX head (R)     the C64 writes at the head
--                reg 3  TX tail (R/W)   the CPU takes bytes to send from the tail
--                reg 4  control: bit 0 = PC connected (R/W), write bit 1 = flush both rings
--                ram    $000-$0FF RX ring, $100-$1FF TX ring
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
    ram_en          : in  std_logic;
    cpu_we          : in  std_logic;
    cpu_addr        : in  unsigned(8 downto 0);
    cpu_wdata       : in  std_logic_vector(7 downto 0);
    cpu_rdata       : out std_logic_vector(7 downto 0);
    cpu_ack         : out std_logic );
end entity;

architecture gideon of ide64_usb is
    signal rx_head      : unsigned(7 downto 0) := (others => '0');
    signal rx_tail      : unsigned(7 downto 0) := (others => '0');
    signal tx_head      : unsigned(7 downto 0) := (others => '0');
    signal tx_tail      : unsigned(7 downto 0) := (others => '0');
    signal connected    : std_logic := '0';

    signal rx_empty     : std_logic;
    signal tx_full      : std_logic;

    signal c64_rd_data  : std_logic;
    signal c64_wr_data  : std_logic;

    signal wr_pend      : std_logic := '0';
    signal wr_addr      : unsigned(7 downto 0) := (others => '0');
    signal wr_data      : std_logic_vector(7 downto 0) := X"00";
    signal b_addr       : unsigned(8 downto 0);
    signal b_rdata      : std_logic_vector(7 downto 0);

    signal reg_rdata    : std_logic_vector(7 downto 0) := X"00";
    signal ram_rdata    : std_logic_vector(7 downto 0);
    signal reg_ack      : std_logic := '0';
    signal ram_ack      : std_logic := '0';
begin
    rx_empty <= '1' when rx_head = rx_tail else '0';
    tx_full  <= '1' when tx_head + 1 = tx_tail else '0';

    process(slot_req, active, rx_empty, tx_full, connected, b_rdata)
    begin
        slot_resp <= c_slot_resp_init;
        if active = '1' then
            if slot_req.bus_address(8 downto 0) = "001011101" and rx_empty = '0' then -- DE5D
                slot_resp.data <= b_rdata;
                slot_resp.reg_output <= '1';
            elsif slot_req.bus_address(8 downto 0) = "001011110" then -- DE5E
                slot_resp.data <= (tx_full or not connected) & (rx_empty or not connected) & "000000";
                slot_resp.reg_output <= '1';
            end if;
        end if;
    end process;

    c64_rd_data <= active and slot_req.io_read  and not rx_empty when slot_req.io_address(8 downto 0) = "001011101" else '0';
    c64_wr_data <= active and slot_req.io_write and connected and not tx_full when slot_req.io_address(8 downto 0) = "001011101" else '0';

    -- Port B reads the RX byte at the tail, except for the clock that writes a TX byte
    b_addr <= ('1' & wr_addr) when wr_pend = '1' else ('0' & rx_tail);

    process(clock)
    begin
        if rising_edge(clock) then
            wr_pend <= '0';
            if c64_rd_data = '1' then
                rx_tail <= rx_tail + 1;
            end if;
            if c64_wr_data = '1' then
                wr_pend <= '1';
                wr_addr <= tx_head;
                wr_data <= slot_req.data;
                tx_head <= tx_head + 1;
            end if;

            reg_ack <= '0';
            if reg_en = '1' then
                reg_ack <= '1';
                if cpu_we = '1' then
                    case to_integer(cpu_addr(2 downto 0)) is
                    when 0 => rx_head <= unsigned(cpu_wdata);
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
                    when 0 => reg_rdata <= std_logic_vector(rx_head);
                    when 1 => reg_rdata <= std_logic_vector(rx_tail);
                    when 2 => reg_rdata <= std_logic_vector(tx_head);
                    when 3 => reg_rdata <= std_logic_vector(tx_tail);
                    when 4 => reg_rdata <= "0000000" & connected;
                    when others => reg_rdata <= X"00";
                    end case;
                end if;
            end if;
            ram_ack <= ram_en;

            if reset = '1' then
                rx_head   <= (others => '0');
                rx_tail   <= (others => '0');
                tx_head   <= (others => '0');
                tx_tail   <= (others => '0');
                connected <= '0';
            end if;
        end if;
    end process;

    cpu_ack   <= reg_ack or ram_ack;
    cpu_rdata <= ram_rdata when ram_ack = '1' else reg_rdata;

    i_ram: entity work.dpram
    generic map (
        g_width_bits    => 8,
        g_depth_bits    => 9,
        g_read_first_a  => false,
        g_read_first_b  => false,
        g_storage       => "block" )
    port map (
        a_clock         => clock,
        a_address       => cpu_addr,
        a_rdata         => ram_rdata,
        a_wdata         => cpu_wdata,
        a_en            => ram_en,
        a_we            => cpu_we,

        b_clock         => clock,
        b_address       => b_addr,
        b_rdata         => b_rdata,
        b_wdata         => wr_data,
        b_en            => '1',
        b_we            => wr_pend );

end architecture;
