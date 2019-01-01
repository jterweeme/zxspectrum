library ieee;
use ieee.std_logic_1164.all;

entity hex7seg is port (
	HEX:	in std_logic_vector(3 downto 0);
	SEGMENT:	out std_logic_vector(6 downto 0)
	);
end hex7seg;

architecture design of hex7seg is
begin
process (hex)
	begin
	case hex is	-- display SEGMENT order a b c d e f g
		when "0000" =>  SEGMENT <= "1000000";		-- 0
		when "0001" =>  SEGMENT <= "1111001";		-- 1 -- need to reverse 
		when "0010" =>  SEGMENT <= "0100100"; 		-- 2
		when "0011" =>  SEGMENT <= "0110000";		-- 3 
		when "0100" =>  SEGMENT <= "0011001";		-- 4 
		when "0101" =>  SEGMENT <= "0010010";		-- 5 
		when "0110" =>  SEGMENT <= "0000010";		-- 6 
		when "0111" =>  SEGMENT <= "1111000";		-- 7 
		when "1000" =>  SEGMENT <= "0000000";		-- 8 
		when "1001" =>  SEGMENT <= "0011000";		-- 9 
		when "1010" =>  SEGMENT <= "0001000";		-- A 
		when "1011" =>  SEGMENT <= "0000011";		-- B 
		when "1100" =>  SEGMENT <= "1000110";		-- C 
		when "1101" =>  SEGMENT <= "0100001";		-- D 
		when "1110" =>  SEGMENT <= "0000110";		-- E 
		when "1111" =>  SEGMENT <= "0001110";		-- F 
		when others =>  SEGMENT <= "0111111";		-- Z
	end case;
	end process;
end design;