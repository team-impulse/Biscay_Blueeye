	
library ieee ;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_misc.all;

library machxo2;
use machxo2.all;

--type registers is array (0 to 15) of std_logic_vector(15 downto 0);

entity ParticleCapture is
port (spi_cs0: in std_logic;
		spi_cs1: in std_logic;
		spi_clock: in std_logic;
		dout: inout std_logic;
		pixels : in std_logic_vector(0 to 15);
		trigd : out std_logic;
		captd : out std_logic;
		clr : in std_logic);
		
end ParticleCapture;

architecture behv_pc of ParticleCapture is
	signal countregs: std_logic_vector(255 downto 0);
	signal is_triggered : std_logic := '0';
	signal trigstate : std_logic := '0';
	signal last_trigstate : std_logic := '0';
	signal hascaptured : std_logic := '0';
	signal pix_counten : std_logic_vector(0 to 15);
	signal eventcount : std_logic_vector(31 downto 0);
	signal intosc : std_logic;
	signal sampleck : std_logic;
	
	signal eventlen : std_logic_vector(15 downto 0);
	
	signal is_valid : std_logic:= '0';

	signal pixel_rst : std_logic := '0';
	signal t_cnten : std_logic := '0';
	signal curstate : std_logic_vector(2 downto 0);

	component SPIRead	
	port (cs0: in std_logic;
		cs1: in std_logic;
		clock: in std_logic;
		dout: inout std_logic;
		regs: in std_logic_vector(255 downto 0);
		pixstate: in std_logic_vector(0 to 15);
		eventlen : in std_logic_vector(15 downto 0);
		eventcount : in std_logic_vector(31 downto 0));
	end component;
	
	component counter
		generic (n: natural := 2);
		port (	clock:	in std_logic;
					clear:	in std_logic;
					count:	in std_logic;
					Q:	out std_logic_vector(n-1 downto 0)
				);
	end component;
	
	component OSCH
	-- synthesis translate_off
	generic (NOM_FREQ: string := "38.00");
	-- synthesis translate_on
	port ( STDBY	: in std_logic;
		   OSC		: out std_logic;
	       SEDSTDBY : out std_logic);
	end component;
	
	component sampleckpll
    port (
        CLKI: in  std_logic; 
        CLKOP: out  std_logic);
	end component;
	
	attribute NOM_FREQ : string;
    attribute NOM_FREQ of internalosc : label is "38.00";

begin
	gen_pixcounter:
		for I in 0 to 15 generate
			pixcounter : counter generic map(16) port map (sampleck, pixel_rst, pix_counten(I), countregs((16*I + 15) downto (16*I)));
		end generate;
	tcounter : counter generic map(16)	port map(sampleck, pixel_rst, t_cnten, eventlen);
	eventcounter : counter generic map(32) port map (is_triggered, clr, '1', eventcount);
	spiif : SPIRead port map(spi_cs0, spi_cs1, spi_clock, dout, countregs, pixels, eventlen, eventcount);
	
	internalosc: OSCH
		-- synthesis translate_off
		GENERIC MAP ( NOM_FREQ => "38.00" )
		-- synthesis translate_on
		PORT MAP ( STDBY=> '0',
		OSC=> intosc );
	
	internalpll : sampleckpll port map(intosc, sampleck);
	
	process(pixels)
		variable oncount : integer range 0 to 16;
	begin
		oncount := 0;
		for I in 0 to 15 loop
			if pixels(I) = '0' then
				oncount := oncount + 1;
			end if;
		end loop;
		
	--	if is_triggered = '0' then
			if oncount >= 5 then
				is_triggered <= '1';
	--		end if;
		else
	--		if oncount < 3 then
				is_triggered <= '0';
			end if;	
	--	end if;		
	end process;
	
	--process(is_triggered, clr)
	--begin
	--	if clr = '1' then
	--		hascaptured <= '0';
	--	elsif (is_triggered = '0' and is_triggered'event) then
	--		hascaptured <= '1';
	--	end if;
	--end process;
	
	--process(is_triggered, hascaptured, clr)
	--begin
	--	if clr = '1' then
	--		trigstate <= '0';
	--	elsif hascaptured = '1' then
	--		trigstate <= '0';
	--	else
	--		trigstate <= is_triggered;
	--	end if;
	--end process;
	
	process(eventlen)
	begin
		if eventlen > 80 then
			is_valid <= '1';
		else
			is_valid <= '0';
		end if;
	end process;
	
	process(sampleck, is_triggered, curstate, clr)
	begin
		if clr = '1' then
			curstate <=  "000";
		else
			if (sampleck = '0' and sampleck'event) then
				case curstate is
					when "000" =>
						captd <= '0';
						if is_triggered = '1' then
							curstate <= "001";
						end if;
					when "001" =>
						if is_triggered = '0' then
							curstate <= "010";
						end if;
					when "010" =>
						if is_valid	= '1' then
							curstate <= "111";
						else
							curstate <= "011";
						end if;
					when "011" =>
						--wait for reset
						curstate <= "100";
					when "100" =>
						curstate <= "000";
					when "111" =>
						captd <= '1';
						curstate <= "111";
					when others =>
						curstate <= "000";
				end case;
			end if;

		end if;
	end process;
	
	process(curstate, clr)
	begin
		if clr = '1' then
			pixel_rst <= '1';
		elsif curstate = "100" then
			pixel_rst <= '1';
		else
			pixel_rst <= '0';
		end if;
	end process;
	
	
	process(pixels, trigstate, curstate, is_triggered)
	begin
		if curstate = "001" and is_triggered = '1' then
			t_cnten <= '1';
			for I in 0 to 15 loop
				pix_counten(I) <= NOT pixels(I);
			end loop;
		else
			t_cnten <= '0';
			for I in 0 to 15 loop
				pix_counten(I) <= '0';
			end loop;
		end if;

	end process;
	
	trigd <= is_triggered;
	--captd <= hascaptured;
end behv_pc;

	


