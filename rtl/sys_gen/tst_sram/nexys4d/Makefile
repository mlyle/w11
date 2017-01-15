# $Id: Makefile 801 2016-08-27 16:47:01Z mueller $
#
#  Revision History: 
# Date         Rev Version  Comment
# XXX NOCOMMIT
#
VBOM_all  = sys_tst_sram_n4d.vbom
BIT_all   = $(VBOM_all:.vbom=.bit)
#
include ${RETROBASE}/rtl/make_viv/viv_default_nexys4d.mk
#
.PHONY : all clean
#
all : $(BIT_all)
#
clean : viv_clean
#
#----
#
include ${RETROBASE}/rtl/make_viv/generic_vivado.mk
#
ifndef DONTINCDEP
include $(VBOM_all:.vbom=.dep_vsyn)
endif
#
