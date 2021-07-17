--This is a comment
--I love comments

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity de2_115 is
port (
    clk50: in std_logic;
    KEY: in std_logic_vector(3 downto 0);
    VGA_R, VGA_G, VGA_B: out std_logic_vector(7 downto 0);
    VGA_HS, VGA_VS, VGA_BLANK_N, VGA_CLK, EAR_OUT: out std_logic;
    PS2_CLK, PS2_DAT: inout std_logic;
    I2C_SCLK, I2C_SDAT: inout std_logic;
    AUD_XCK, AUD_BCLK, AUD_ADCLRCK, AUD_DACLRCK, AUD_DACDAT: out std_logic;
    AUD_ADCDAT: in std_logic;
    EAR_IN: in std_logic);
end entity de2_115;

architecture rtl of de2_115 is

component i2s_intf is
    generic(
      mclk_rate : positive := 12000000;
      sample_rate : positive := 8000;
      preamble : positive := 1; -- I2S
      word_length : positive := 16);
    port (
      CLK, nRESET: in std_logic;
      PCM_INL, PCM_INR: out std_logic_vector(word_length - 1 downto 0);
      PCM_OUTL, PCM_OUTR: in std_logic_vector(word_length - 1 downto 0);
      I2S_MCLK, I2S_LRCLK, I2S_BCLK, I2S_DOUT: out std_logic;
      I2S_DIN: in std_logic);
    end component;

component i2c_loader is
    generic (
      device_address: integer := 16#1a#;
      num_retries: integer := 0;
      log2_divider: integer := 6);
    port (
      CLK, nRESET: in std_logic;
      I2C_SCL, I2C_SDA: inout std_logic);
    end component;

signal reset_n: std_logic;
	 
--klok signalen
signal audio_clock, clk28: std_logic;

signal pll_reset, pll_locked, cpu_en, vid_en, cpu_mreq_n: std_logic;
signal ula_en, rom_en, ram_en, vid_irq_n, ram_wr, cpu_ioreq_n, cpu_wr_n: std_logic;
signal ula_border: std_logic_vector(2 downto 0);
signal ram_in, ram_out, rom_di, cpu_di, cpu_do, ula_do: std_logic_vector(7 downto 0);
signal cpu_a, ram_a: std_logic_vector(15 downto 0);
signal vid_a: std_logic_vector(12 downto 0);
signal xkeyb: std_logic_vector(4 downto 0);
signal counter: unsigned(19 downto 0);
signal pcm_outl, pcm_outr, pcm_inl, pcm_inr: std_logic_vector(15 downto 0);
signal ula_ear_out, ula_mic_out, ula_ear_in: std_logic;
signal pcm_lrclk: std_logic;
begin
    pll: entity work.pll_main port map (pll_reset, clk50, clk28, audio_clock, pll_locked);
	 
    pll_reset <= not KEY(0);
	 
    reset_n <= not (pll_reset or not pll_locked);
	 
    cpu_en <= not (counter(0) or counter(1) or counter(2));
	 
    vid_en <= counter(0);
	 
	 EAR_OUT <= ula_ear_out;
	 pcm_outl <= ula_ear_out & "000000000000000";
	 pcm_outr <= ula_ear_out & "000000000000000";
	 AUD_DACLRCK <= pcm_lrclk;
    AUD_ADCLRCK <= pcm_lrclk;
	 
	 i2s: i2s_intf port map (
        audio_clock, reset_n,
        pcm_inl, pcm_inr,
        pcm_outl, pcm_outr,
        AUD_XCK, pcm_lrclk,
        AUD_BCLK, AUD_DACDAT, AUD_ADCDAT);

    i2c: i2c_loader 
        generic map (log2_divider => 7)
        port map (clk28, reset_n, I2C_SCLK, I2C_SDAT);

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
            ula_ear_out <= '0';
            ula_mic_out <= '0';
            ula_border <= (others => '0');
            ula_do <= (others => '0');
        elsif rising_edge(clk28) then
            ula_do <= '0' & ula_ear_in & '0' & xkeyb;
            if ula_en = '1' and cpu_wr_n = '0' then
                ula_ear_out <= cpu_do(4);
                ula_mic_out <= cpu_do(3);
                ula_border <= cpu_do(2 downto 0);
            end if;
        end if;
    end process;
    
    -- Hysteresis for EAR input (should help reliability)
    process (clk28)
         variable in_val : integer;
    begin
        in_val := to_integer(signed(pcm_inl));

        if rising_edge(clk28) then
            if in_val < -15 then
                ula_ear_in <= '0';
            elsif in_val > 15 then
                ula_ear_in <= '1';
            end if;
        end if;
    end process;
end architecture rtl;


