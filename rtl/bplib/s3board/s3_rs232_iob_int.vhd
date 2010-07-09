-- $Id: s3_rs232_iob_int.vhd 314 2010-07-09 17:38:41Z mueller $
--
-- Copyright 2010- by Walter F.J. Mueller <W.F.J.Mueller@gsi.de>
--
-- This program is free software; you may redistribute and/or modify it under
-- the terms of the GNU General Public License as published by the Free
-- Software Foundation, either version 2, or at your option any later version.
--
-- This program is distributed in the hope that it will be useful, but
-- WITHOUT ANY WARRANTY, without even the implied warranty of MERCHANTABILITY
-- or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
-- for complete details.
--
------------------------------------------------------------------------------
-- Module Name:    s3_rs232_iob_int - syn
-- Description:    iob's for internal rs232
--
-- Dependencies:   xlib/iob_reg_i
--                 xlib/iob_reg_o
--
-- Test bench:     -
--
-- Target Devices: generic
-- Tool versions:  xst 11.4; ghdl 0.26
--
-- Revision History: 
-- Date         Rev Version  Comment
-- 2010-04-17   278   1.0    Initial version
------------------------------------------------------------------------------
--    

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

use work.slvtypes.all;
use work.xlib.all;

-- ----------------------------------------------------------------------------

entity s3_rs232_iob_int is              -- iob's for internal rs232
  port (
    CLK : in slbit;                     -- clock
    RXD : out slbit;                    -- receive data (board view)
    TXD : in slbit;                     -- transmit data (board view)
    I_RXD : in slbit;                   -- pad-i: receive data (board view)
    O_TXD : out slbit                   -- pad-o: transmit data (board view)
  );
end s3_rs232_iob_int;

architecture syn of s3_rs232_iob_int is
begin

  IOB_RXD : iob_reg_i                   -- line idle=1, so init sync flop =1
    generic map (INIT => '1')
    port map (CLK => CLK, CE => '1', DI => RXD, PAD => I_RXD);
  
  IOB_TXD : iob_reg_o                   -- line idle=1, so init sync flop =1
    generic map (INIT => '1')
    port map (CLK => CLK, CE => '1', DO => TXD, PAD => O_TXD);

end syn;