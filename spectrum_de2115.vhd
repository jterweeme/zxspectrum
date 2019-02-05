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
generic (
    ROM_OFFSET: std_logic_vector(7 downto 0) := "00000000"
    );

port (
    clk50: in std_logic;
    SW: in std_logic_vector(9 downto 0);
	 KEY: in std_logic_vector(3 downto 0);
    HEX0, HEX1, HEX2, HEX3: out std_logic_vector(6 downto 0);
    LEDR: out std_logic_vector(17 downto 0);
    LEDG: out std_logic_vector(7 downto 0);
    VGA_R, VGA_G, VGA_B: out std_logic_vector(7 downto 0);
    VGA_HS, VGA_VS: out std_logic;
    VGA_BLANK_N, VGA_CLK: out std_logic;
    UART_TXD: out std_logic;
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
    GPIO: out std_logic_vector(34 downto 0);
	 EAR_IN: in std_logic
    );
end entity;

architecture rtl of spectrum_de2115 is
component pll_main IS
    PORT
    (
        areset: in std_logic := '0';
        inclk0: in std_logic := '0';
        c0: out std_logic;
        c1: out std_logic;
        locked: out std_logic 
    );
end component;

component clocks is
port (
    CLK: in std_logic;
    nRESET: in std_logic;
    CLKEN_CPU: out std_logic;
    CLKEN_VID: out std_logic
    );
end component;

component T80se is
    generic(
        Mode: integer := 0;    -- 0 => Z80, 1 => Fast Z80, 2 => 8080, 3 => GB
        T2Write: integer := 0;  -- 0 => WR_n active in T3, /=0 => WR_n active in T2
        IOWait: integer := 1   -- 0 => Single cycle I/O, 1 => Std I/O cycle
    );
    port(
        RESET_n: in std_logic;
        CLK_n: in std_logic;
        CLKEN: in std_logic;
        WAIT_n: in std_logic;
        INT_n: in std_logic;
        NMI_n: in std_logic;
        BUSRQ_n: in std_logic;
        MREQ_n: out std_logic;
        IORQ_n: out std_logic;
        WR_n: out std_logic;
        A: out std_logic_vector(15 downto 0);
        DI: in std_logic_vector(7 downto 0);
        DO: out std_logic_vector(7 downto 0)
    );
end component;

component ula_port is
port (
    CLK: in std_logic;
    nRESET: in std_logic;
    D_IN: in std_logic_vector(7 downto 0);
    D_OUT: out std_logic_vector(7 downto 0);
    ENABLE: in std_logic;
    nWR: in std_logic;
    BORDER_OUT: out std_logic_vector(2 downto 0);
    EAR_OUT, MIC_OUT: out std_logic;
    KEYB_IN: in std_logic_vector(4 downto 0);
    EAR_IN: in std_logic
    );
end component;

component video is
port(
    CLK: in std_logic;
    CLKEN: in std_logic;
    nRESET: in std_logic;
    VID_A: out std_logic_vector(12 downto 0);
    VID_D_IN: in std_logic_vector(7 downto 0);
    BORDER_IN: in std_logic_vector(2 downto 0);
    R, G, B: out std_logic_vector(7 downto 0);
    nVSYNC, nHSYNC, nCSYNC, nHCSYNC: out std_logic;
    IS_BORDER: out std_logic;
    IS_VALID: out std_logic;
    PIXCLK: out std_logic;
    FLASHCLK: out std_logic;
    nIRQ: out std_logic
);
end component;

component keyboard is
port (
    CLK, nRESET: in std_logic;
    PS2_CLK, PS2_DATA: inout std_logic;
    A: in std_logic_vector(15 downto 0);
    KEYB: out std_logic_vector(4 downto 0)
    );
end component;

signal pll_reset, pll_locked: std_logic;
signal clk28, clk3_5, clk14: std_logic;
signal audio_clock: std_logic;
signal reset_n: std_logic;
signal ula_enable, rom_enable, ram_enable: std_logic;
signal page_rom_sel: std_logic := '0'; -- bit 4
signal ram_page: std_logic_vector(2 downto 0);
signal sram_di: std_logic_vector(7 downto 0);
signal cpu_wait_n, cpu_irq_n, cpu_nmi_n, cpu_busreq_n, cpu_mreq_n, cpu_ioreq_n: std_logic;
signal cpu_wr_n: std_logic;
signal cpu_a: std_logic_vector(15 downto 0);
signal cpu_di, cpu_do, ula_do: std_logic_vector(7 downto 0);
signal ula_border: std_logic_vector(2 downto 0);
signal ula_ear_out, ula_mic_out, ula_ear_in: std_logic;
signal vid_a: std_logic_vector(12 downto 0);
signal vid_di: std_logic_vector(7 downto 0);
signal vid_r_out, vid_g_out, vid_b_out: std_logic_vector(7 downto 0);
signal vid_vsync_n, vid_hsync_n, vid_csync_n, vid_hcsync_n: std_logic;
signal vid_is_border, vid_is_valid, vid_pixclk, vid_flashclk, vid_irq_n: std_logic;
signal keyb: std_logic_vector(4 downto 0);
begin
    pll: pll_main port map (pll_reset, clk50, clk28, audio_clock, pll_locked);
    clken: clocks port map (clk28, reset_n, clk3_5, clk14);
	 
    cpu: T80se port map (
        reset_n, clk28, clk3_5, 
        cpu_wait_n, cpu_irq_n, cpu_nmi_n,
        cpu_busreq_n, cpu_mreq_n, cpu_ioreq_n,
		  cpu_wr_n, cpu_a, cpu_di, cpu_do
        );
    
    cpu_irq_n <= vid_irq_n; -- VSYNC interrupt routed to CPU
    cpu_wait_n <= '1';
    cpu_nmi_n <= '1';
    cpu_busreq_n <= '1';

    kb: keyboard port map (clk28, reset_n, PS2_CLK, PS2_DAT, cpu_a, keyb);

--    ula: ula_port port map (
--        clk28, reset_n, cpu_do, ula_do,
--        ula_enable, cpu_wr_n, ula_border,
--        ula_ear_out, ula_mic_out,
--        keyb, EAR_IN);

    process (clk28, reset_n)
    begin
        if reset_n = '0' then
			  ula_ear_out <= '0';
			  ula_mic_out <= '0';
			  ula_border <= (others => '0');
			  ula_do <= (others => '0');
        elsif rising_edge(clk28) then
			  ula_do <= '0' & EAR_IN & '0' & keyb;
			  if ula_enable = '1' and cpu_wr_n = '0' then
				  ula_ear_out <= cpu_do(4);
				  ula_mic_out <= cpu_do(3);
				  ula_border <= cpu_do(2 downto 0);
			  end if;
        end if;
    end process;		  
		  
    vid: video port map (
        clk28, clk14, reset_n,
        vid_a, vid_di, ula_border,
        vid_r_out, vid_g_out, vid_b_out,
        vid_vsync_n, vid_hsync_n,
        vid_csync_n, vid_hcsync_n,
        vid_is_border, vid_is_valid,
        vid_pixclk, vid_flashclk,
        vid_irq_n);

    pll_reset <= not KEY(0);
    reset_n <= not (pll_reset or not pll_locked);
    ula_enable <= (not cpu_ioreq_n) and not cpu_a(0); -- all even IO addresses
    rom_enable <= (not cpu_mreq_n) and not (cpu_a(15) or cpu_a(14));
    ram_enable <= not (cpu_mreq_n or rom_enable);
    -- 128K has pageable RAM at 0xc000
    ram_page <=
            "000" when cpu_a(15 downto 14) = "11" else -- Selectable bank at 0xc000
            cpu_a(14) & cpu_a(15 downto 14); -- A=bank: 00=XXX, 01=101, 10=010, 11=XXX

    -- CPU data bus mux
    cpu_di <=
        sram_di when ram_enable = '1' else
        FL_DQ when rom_enable = '1' else
        ula_do when ula_enable = '1' else
        (others => '1');

    FL_RST_N <= reset_n;
    FL_CE_N <= '0';
    FL_OE_N <= '0';
    FL_WE_N <= '1';
    FL_ADDR <= '0' & ROM_OFFSET & cpu_a(13 downto 0);
    SRAM_CE_N <= '0';
    SRAM_OE_N <= '0';
    sram_di <= SRAM_DQ(15 downto 8) when cpu_a(0) = '1' else SRAM_DQ(7 downto 0);
    vid_di <= SRAM_DQ(15 downto 8) when vid_a(0) = '1' else SRAM_DQ(7 downto 0);

    -- Synchronous outputs to SRAM
    process (clk28, reset_n)
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

            -- Register SRAM signals to outputs (clock must be at least 2x CPU clock)
            if clk14 = '1' then
                -- Fetch data from previous CPU cycle
                -- Select upper or lower byte depending on LSb of address
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
                -- Fetch data from previous display cycle
                -- Because we have time division instead of bus contention
                -- we don't bother using the vid_rd_n signal from the ULA
                -- No writes here so just enable both upper and lower bytes and let
                -- the bus mux select the right one
                SRAM_UB_N <= '0';
                SRAM_LB_N <= '0';
                SRAM_WE_N <= '1';
                SRAM_ADDR <= "00001010" & vid_a(12 downto 1);
            end if;
        end if;
    end process;

    VGA_R <= vid_r_out;
    VGA_G <= vid_g_out;
    VGA_B <= vid_b_out;
    VGA_HS <= vid_hcsync_n;
    VGA_VS <= vid_vsync_n;
    VGA_BLANK_N <= vid_is_valid;
    VGA_CLK <= vid_pixclk;
    GPIO(1) <= ula_ear_out;
    --GPIO <= "000000000000000000000000000000000" & ula_ear_out & ula_ear_out;
    HEX0 <= "0000000";
    HEX1 <= "1111000";
    HEX2 <= "0000010";
    HEX3 <= "0000000";
--    HEX4 <= "0000000";
--    HEX5 <= "0000000";
--    HEX6 <= "0000000";
--    HEX7 <= "1111001";
    LEDG <= "11000000";
	 LEDR <= "000000000011111111";
    AUD_DACLRCK <= '1';
    AUD_ADCLRCK <= '1';
    UART_TXD <= '1';
end architecture;


