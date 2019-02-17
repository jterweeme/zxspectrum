-- ZX Spectrum for Altera DE2-115
--
-- Copyright (c) 2019 Jasper ter Weeme
-- Copyright (c) 2014 Stephen Eddy
-- Modified from code portions Copyright (c) 2009-2011 Mike Stirling
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- * Redistributions of source code must retain the above copyright notice,
--   this list of conditions and the following disclaimer.
--
-- * Redistributions in synthesized form must reproduce the above copyright
--   notice, this list of conditions and the following disclaimer in the
--   documentation and/or other materials provided with the distribution.
--
-- * Neither the name of the author nor the names of other contributors may
--   be used to endorse or promote products derived from this software without
--   specific prior written agreement from the author.
--
-- * License is granted for non-commercial use only.  A fee may not be charged
--   for redistributions as source code or in synthesized/hardware form without 
--   specific prior written agreement from the author.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- Sinclair ZX Spectrum
--
-- Terasic DE2-115 top-level
--
-- (C) 2014 Stephen Eddy

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spectrum_de2115 is
port (
    clk50: in std_logic;
    SW: in std_logic_vector(9 downto 0);
    KEY: in std_logic_vector(3 downto 0);
    HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7: out std_logic_vector(6 downto 0);
    LEDR: out std_logic_vector(17 downto 0);
    LEDG: out std_logic_vector(7 downto 0);
    VGA_R, VGA_G, VGA_B: out std_logic_vector(7 downto 0);
    VGA_HS, VGA_VS, VGA_BLANK_N, VGA_CLK: out std_logic;
    UART_TXD, UART_RTS: out std_logic;
    PS2_CLK, PS2_DAT: inout std_logic;
    I2C_SCLK, I2C_SDAT: inout std_logic;
    AUD_XCK, AUD_BCLK, AUD_ADCLRCK, AUD_DACLRCK, AUD_DACDAT: out std_logic;
    AUD_ADCDAT: in std_logic;
    SRAM_ADDR: out std_logic_vector(19 downto 0);
    SRAM_DQ: inout std_logic_vector(15 downto 0);
    SRAM_CE_N, SRAM_OE_N, SRAM_WE_N, SRAM_UB_N, SRAM_LB_N: out std_logic;
    FL_ADDR: out std_logic_vector(22 downto 0);
    FL_DQ: inout std_logic_vector(7 downto 0);
    FL_RST_N, FL_OE_N, FL_WE_N, FL_CE_N: out std_logic;
    GPIO: inout std_logic_vector(12 downto 0);
    GPIO2: out std_logic_vector(15 downto 0);
    GPIO3: inout std_logic_vector(4 downto 0);
    EAR_OUT: inout std_logic;
    EAR_IN: in std_logic;
    EX_IO: inout std_logic_vector(6 downto 0);
    ENET0_GTX_CLK, ENET0_INT_N, ENET0_MDC, ENET0_MDIO: out std_logic;
    ENET0_LINK100: in std_logic
    --LCD_DATA: out std_logic_vector(7 downto 0)
    );
end entity;

architecture rtl of spectrum_de2115 is
signal pll_reset, pll_locked, clk28, cpu_en, vid_en, reset_n: std_logic;
signal ula_enable, rom_enable, ram_enable: std_logic;
signal ram_page, ula_border: std_logic_vector(2 downto 0);
signal sram_di, vid_di, rom_di, cpu_di, cpu_do, ula_do: std_logic_vector(7 downto 0);
signal cpu_mreq_n, cpu_ioreq_n, cpu_wr_n, ula_ear_in: std_logic;
signal cpu_a: std_logic_vector(15 downto 0);
signal vid_a: std_logic_vector(12 downto 0);
signal vid_is_valid, vid_pixclk, vid_irq_n: std_logic;
signal keyb: std_logic_vector(4 downto 0);
signal counter: unsigned(19 downto 0);
begin
    pll: entity work.pll_main port map (pll_reset, clk50, clk28, pll_locked);
    cpu_en <= not (counter(0) or counter(1) or counter(2));
    vid_en <= counter(0);

    process (reset_n, clk28)
    begin
        if reset_n = '0' then
            counter <= (others => '0');
        elsif falling_edge(clk28) then
            counter <= counter + 1;
        end if;
    end process;

    romx: entity work.rom port map (cpu_a(13 downto 0), clk28, rom_di);

    cpu: entity work.T80se port map (reset_n, clk28, cpu_en, '1', vid_irq_n, '1',
            '1', MREQ_n => cpu_mreq_n, IORQ_n => cpu_ioreq_n,
				WR_n => cpu_wr_n, A => cpu_a, DI => cpu_di, DO => cpu_do);

    kb: entity work.keyboard port map (clk28, reset_n, PS2_CLK, PS2_DAT, cpu_a, keyb);

    vid: entity work.video port map (
        clk28, vid_en, reset_n, vid_a, vid_di, ula_border,
        VGA_R, VGA_G, VGA_B, VGA_VS, VGA_HS,
        VGA_BLANK_N, VGA_CLK, vid_irq_n);

    SRAM_CE_N <= '0';
    SRAM_OE_N <= '0';
    pll_reset <= not KEY(0);
    reset_n <= not (pll_reset or not pll_locked);
    ula_enable <= (not cpu_ioreq_n) and not cpu_a(0); -- all even IO addresses
    rom_enable <= (not cpu_mreq_n) and not (cpu_a(15) or cpu_a(14));
    ram_enable <= not (cpu_mreq_n or rom_enable);
    ram_page <= "000" when cpu_a(15 downto 14) = "11" else cpu_a(14) & cpu_a(15 downto 14);
    sram_di <= SRAM_DQ(15 downto 8) when cpu_a(0) = '1' else SRAM_DQ(7 downto 0);
    vid_di <= SRAM_DQ(15 downto 8) when vid_a(0) = '1' else SRAM_DQ(7 downto 0);

    cpu_mux: cpu_di <= sram_di when ram_enable = '1' else
        rom_di when rom_enable = '1' else
        ula_do when ula_enable = '1' else
        (others => '1');

    process (clk28, reset_n, ram_enable, cpu_wr_n)
    variable sram_write: std_logic;
    begin
        sram_write := ram_enable and not cpu_wr_n;
        if reset_n = '0' then
            SRAM_WE_N <= '1';
            SRAM_UB_N <= '1';
            SRAM_LB_N <= '1';
            SRAM_DQ <= (others => 'Z');
        elsif rising_edge(clk28) then
            SRAM_DQ <= (others => 'Z');
            if vid_en = '1' then
                SRAM_UB_N <= not cpu_a(0);
                SRAM_LB_N <= cpu_a(0);
                SRAM_WE_N <= not sram_write;
                if rom_enable = '0' then
                    SRAM_ADDR <= "0000" & ram_page & cpu_a(13 downto 1);
                end if;
                if sram_write = '1' then
                    SRAM_DQ(15 downto 8) <= cpu_do;
                    SRAM_DQ(7 downto 0) <= cpu_do;
                end if;
            else
                SRAM_UB_N <= '0';
                SRAM_LB_N <= '0';
                SRAM_WE_N <= '1';
                SRAM_ADDR <= "00001010" & vid_a(12 downto 1);
            end if;
        end if;
    end process;

    ula_port: process (clk28, reset_n) begin
        if reset_n = '0' then
		      EAR_OUT <= '0';
            --ula_mic_out <= '0';
            ula_border <= (others => '0');
            ula_do <= (others => '0');
        elsif rising_edge(clk28) then
            ula_do <= '0' & EAR_IN & '0' & keyb;
            if ula_enable = '1' and cpu_wr_n = '0' then
                EAR_OUT <= cpu_do(4);
                --ula_mic_out <= cpu_do(3);
                ula_border <= cpu_do(2 downto 0);
            end if;
        end if;
    end process;	 

    xhex7: entity work.encoder port map("0001", HEX7);
	 xhex6: entity work.encoder port map("0010", HEX6);
	 xhex5: entity work.encoder port map("0011", HEX5);
	 xhex4: entity work.encoder port map("0100", HEX4);
	 xhex3: entity work.encoder port map("0101", HEX3);
	 xhex2: entity work.encoder port map("0110", HEX2);
	 xhex1: entity work.encoder port map("0111", HEX1);
	 xhex0: entity work.encoder port map("1000", HEX0);	 
    GPIO <= "0000000000000";
    GPIO2 <= cpu_a;
    FL_RST_N <= '0';
    FL_CE_N <= '0';
    FL_OE_N <= '0';
    FL_WE_N <= '1';
    FL_ADDR <= (others => '0');
    LEDG <= "11000000";
    LEDR <= "000000000011111111";
    AUD_DACLRCK <= '1';
    AUD_ADCLRCK <= '1';
    UART_TXD <= '1';
    UART_RTS <= '1';
    AUD_XCK <= '1';
    AUD_BCLK <= '1';
    AUD_DACDAT <= '1';
    EX_IO <= "0000000";
    ENET0_GTX_CLK <= '0';
    ENET0_INT_N <= '0';
    ENET0_MDC <= '0';
    ENET0_MDIO <= '0';
end architecture;


