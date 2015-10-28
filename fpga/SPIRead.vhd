library ieee ;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity SPIRead is

port (cs0: in std_logic;
		cs1: in std_logic;
		clock: in std_logic;
		dout: inout std_logic;
		regs: in std_logic_vector(255 downto 0);
		pixstate: in std_logic_vector(0 to 15);
		eventlen : in std_logic_vector(15 downto 0);
		eventcount : in std_logic_vector(31 downto 0));
		
end SPIRead;

architecture behv_spi of SPIRead is
	constant VERSION : unsigned := x"1234";

	signal curbit : std_logic_vector(8 downto 0);
	signal rst: std_logic;
	component counter
		generic (n: natural := 2);
		port (	clock:	in std_logic;
					clear:	in std_logic;
					count:	in std_logic;
					Q:	out std_logic_vector(n-1 downto 0)
				);
	end component;
	
begin
	bitcounter : counter generic map (9) port map (clock, rst, '1', curbit);
	process(cs0, cs1)
	begin
		rst <= cs0 and cs1;
	end process;
	process(cs0, cs1, curbit, regs, pixstate)
	begin
		if cs0 = '0' then
			if curbit(8) = '0' then
				dout <= regs (to_integer(unsigned(curbit(7 downto 0))));
			else
				dout <= eventlen(to_integer(unsigned(curbit(3 downto 0))));
			end if;
		elsif cs1 = '0' then
			case curbit(5 downto 4) is
				when "00" => dout <= pixstate(to_integer(unsigned(curbit(3 downto 0))));
				when "01" => dout <= VERSION(to_integer(unsigned(curbit(3 downto 0))));
				when "10" => dout <= eventcount(to_integer(unsigned( curbit(4 downto 0))));
				when "11" => dout <= eventcount(to_integer(unsigned( curbit(4 downto 0))));
				when others => dout <= '0';
			end case;
		else
			dout <= 'Z';	
		end if;
	end process;
end behv_spi;