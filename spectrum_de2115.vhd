library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spectrum_de2115 is
port (
    clk50: in std_logic;
    KEY: in std_logic_vector(3 downto 0);
    KEYB: in std_logic_vector(4 downto 0);
    A, VGA_R, VGA_G, VGA_B: out std_logic_vector(7 downto 0);
    VGA_HS, VGA_VS, VGA_BLANK_N, VGA_CLK, EAR_OUT: out std_logic;
    PS2_CLK, PS2_DAT: inout std_logic;
    EAR_IN: in std_logic);
end entity;

architecture rtl of spectrum_de2115 is
signal pll_reset, pll_locked, clk28, cpu_en, vid_en, reset_n, cpu_mreq_n: std_logic;
signal ula_en, rom_en, ram_en, vid_irq_n, ram_wr, cpu_ioreq_n, cpu_wr_n: std_logic;
signal ula_border: std_logic_vector(2 downto 0);
signal ram_in, ram_out, rom_di, cpu_di, cpu_do, ula_do: std_logic_vector(7 downto 0);
signal cpu_a, ram_a: std_logic_vector(15 downto 0);
signal vid_a: std_logic_vector(12 downto 0);
signal xkeyb: std_logic_vector(4 downto 0);
signal counter: unsigned(19 downto 0);
begin
    A <= cpu_a(15 downto 8);
    pll: entity work.pll_main port map (pll_reset, clk50, clk28, pll_locked);
    pll_reset <= not KEY(0);
    reset_n <= not (pll_reset or not pll_locked);
    cpu_en <= not (counter(0) or counter(1) or counter(2));
    vid_en <= counter(0);

    process (reset_n, clk28) begin
        if reset_n = '0' then
            counter <= (others => '0');
        elsif falling_edge(clk28) then
            counter <= counter + 1;
        end if;
    end process;

    cpu: entity work.T80se port map (reset_n, clk28, cpu_en, '1', vid_irq_n, '1',
            '1', MREQ_n => cpu_mreq_n, IORQ_n => cpu_ioreq_n,
            WR_n => cpu_wr_n, A => cpu_a, DI => cpu_di, DO => cpu_do);

    vid: entity work.video port map (clk28, vid_en, reset_n, vid_a, ram_out, ula_border,
        VGA_R, VGA_G, VGA_B, VGA_VS, VGA_HS, VGA_BLANK_N, VGA_CLK, vid_irq_n);

    romx: entity work.rom port map (cpu_a(13 downto 0), clk28, rom_di);
    ramx: entity work.ram port map (ram_a, clk28, ram_in, ram_wr, ram_out);
    kb: entity work.keyboard port map (clk28, reset_n, PS2_CLK, PS2_DAT, cpu_a, xkeyb);
    ula_en <= not cpu_ioreq_n and not cpu_a(0); -- all even IO addresses
    rom_en <= not cpu_mreq_n and not (cpu_a(15) or cpu_a(14));
    ram_en <= not (cpu_mreq_n or rom_en);

    cpu_mux: cpu_di <= ram_out when ram_en = '1' else
        rom_di when rom_en = '1' else
        ula_do when ula_en = '1' else
        (others => '1');

    process (clk28, reset_n, ram_en, cpu_wr_n) begin
        if rising_edge(clk28) then
            if vid_en = '1' then
                ram_wr <= ram_en and not cpu_wr_n;
                ram_a <= cpu_a(15 downto 0);
                ram_in <= cpu_do;
            else
                ram_wr <= '0';
                ram_a <= "010" & vid_a(12 downto 0);
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
            ula_do <= '0' & EAR_IN & '0' & KEYB;
            if ula_en = '1' and cpu_wr_n = '0' then
                EAR_OUT <= cpu_do(4);
                --ula_mic_out <= cpu_do(3);
                ula_border <= cpu_do(2 downto 0);
            end if;
        end if;
    end process;
end architecture;


