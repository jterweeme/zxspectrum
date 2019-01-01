library ieee;
use ieee.std_logic_1164.all;

entity bcd7seg is port (
	bcd:	in std_logic_vector(3 downto 0);
	segment:	out std_logic_vector(6 downto 0)
	);
end bcd7seg;

architecture design of bcd7seg is
begin
process (bcd)
	begin
	case bcd is	-- display segment order a b c d e f g
		when "0000" =>  segment < = "1111110";
		when "0001" =>  segment < = "0110000"; 
		when "0010" =>  segment < = "1101101"; 
		when "0011" =>  segment < = "1111001"; 
		when "0100" =>  segment < = "0110011"; 
		when "0101" =>  segment < = "1011011"; 
		when "0110" =>  segment < = "1011111"; 
		when "0111" =>  segment < = "1110000"; 
		when "1000" =>  segment < = "1111111"; 
		when "1001" =>  segment < = "1110011"; 
		when others =>  segment < = "0000001";
	end case;
	end process;
end design;