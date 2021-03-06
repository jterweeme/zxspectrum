-- ZX Spectrum for Altera DE1
--
-- Copyright (c) 2019 Jasper ter Weeme
-- Copyright (c) 2009-2011 Mike Stirling
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

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity video is
port(
    CLK, CLKEN, nRESET: in std_logic;
    VID_A: out std_logic_vector(12 downto 0);
    VID_D_IN: in std_logic_vector(7 downto 0);
    BORDER_IN: in std_logic_vector(2 downto 0);
    R, G, B: out std_logic_vector(7 downto 0);
    nVSYNC, nHCSYNC: out std_logic;
    IS_VALID: out std_logic;
    PIXCLK: out std_logic;
    nIRQ: out std_logic
);
end video;

architecture video_arch of video is
signal pixels: std_logic_vector(9 downto 0);
signal hcounter, vcounter: unsigned(9 downto 0);
signal attr: std_logic_vector(7 downto 0);
signal flashcounter: unsigned(4 downto 0);
signal vblanking, hblanking: std_logic;
signal hpicture, vpicture: std_logic;
signal picture: std_logic;
signal blanking: std_logic;
signal hsync, vsync: std_logic;
signal red, green, blue: std_logic;
signal bright: std_logic;
signal dot: std_logic;
begin
    picture <= hpicture and vpicture;
    blanking <= hblanking or vblanking;
    IS_VALID <= not blanking;
    PIXCLK <= CLK and CLKEN and nRESET;
    nVSYNC <= not vsync;
    nHCSYNC <= not hsync;
    dot <= pixels(9) xor (flashcounter(4) and attr(7));

    red <= attr(1) when picture = '1' and dot = '1' else
        attr(4) when picture = '1' and dot = '0' else
        BORDER_IN(1) when blanking = '0' else
        '0';
    green <= attr(2) when picture = '1' and dot = '1' else
        attr(5) when picture = '1' and dot = '0' else
        BORDER_IN(2) when blanking = '0' else
        '0';
    blue <= attr(0) when picture = '1' and dot = '1' else
        attr(3) when picture = '1' and dot = '0' else
        BORDER_IN(0) when blanking = '0' else
        '0';
    bright <= attr(6) when picture = '1' else
        '0';

    process(nRESET,CLK)
    begin
        if nRESET = '0' then
            R <= (others => '0');
            G <= (others => '0');
            B <= (others => '0');
        elsif rising_edge(CLK) then
            R <= (7 => red, others => bright and red);
            G <= (7 => green, others => bright and green);
            B <= (7 => blue, others => bright and blue);
        end if;
    end process;

    VID_A(12 downto 0) <=
        std_logic_vector(vcounter(8 downto 7) & vcounter(3 downto 1) & vcounter(6 downto 4) & hcounter(8 downto 4))
        when hcounter(2) = '0' else
        "110" & std_logic_vector(vcounter(8 downto 4) & hcounter(8 downto 4));

    vpicture <= not (vcounter(9) or (vcounter(8) and vcounter(7)));

    process(nRESET,CLK,CLKEN,hcounter,vcounter)
    begin
        if nRESET = '0' then
            hcounter <= (others => '0');
            vcounter <= (others => '0');
            flashcounter <= (others => '0');
            vblanking <= '0';
            hblanking <= '0';
            hpicture <= '1';
            hsync <= '0';
            vsync <= '0';
            nIRQ <= '1';
            pixels <= (others => '0');
            attr <= (others => '0');
        elsif rising_edge(CLK) and CLKEN = '1' then
            if vpicture = '1' and hcounter(0) = '0' then
                pixels(9 downto 1) <= pixels(8 downto 0);
                if hcounter(9) = '0' and hcounter(3) = '0' and hcounter(1) = '1' then
                    if hcounter(2) = '0' then
                        pixels(7 downto 0) <= VID_D_IN;
                    else
                        attr <= VID_D_IN;
                    end if;
                end if;
                if hcounter(9) = '0' and hcounter(2 downto 1) = "11" then
                    hpicture <= '1';
                end if;
                if hcounter(9) = '1' and hcounter(2 downto 1) = "11" then
                    hpicture <= '0';
                end if;
            end if;
            if hcounter = "1101111110" then
                hcounter <= (others => '0');
                vcounter <= vcounter + 1;
            else
                hcounter <= hcounter + 2;
                hcounter(0) <= '0';
            end if;
            case hcounter(9 downto 4) is
            when "100110" => hblanking <= '1';
            when "101001" => hsync <= '1';
            when "101101" => hsync <= '0';
            when "110010" => hblanking <= '0';
            when others => null;
            end case;
            if hcounter(7) = '1' then
                nIRQ <= '1';
            end if;
            case vcounter(9 downto 3) is
            when "0111110" =>
                vblanking <= '1';
                vsync <= '1';
                nIRQ <= '0';
            when "0111111" =>
                vsync <= '0';
            when "1000000" =>
                vblanking <= '0';
            when others =>
                null;
            end case;
            if vcounter(9 downto 1) = "100110111" then
                if (vcounter(0) = '1') then
                    vcounter <= (others => '0');
                    flashcounter <= flashcounter + 1;
                end if;
            end if;
        end if;
    end process;
end video_arch;





