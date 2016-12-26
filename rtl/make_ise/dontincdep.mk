# $Id: dontincdep.mk 830 2016-12-26 20:25:49Z mueller $
#
# Copyright 2013- by Walter F.J. Mueller <W.F.J.Mueller@gsi.de>
# License disclaimer see License.txt in $RETROBASE directory
#
#  Revision History: 
# Date         Rev Version  Comment
# 2013-01-27   477   1.0    Initial version
#
# DONTINCDEP controls whether dependency files are included. Set it if
# any of the 'clean' type targets is involved
#
ifneq  ($(findstring clean, $(MAKECMDGOALS)),)
DONTINCDEP = 1
endif
ifneq  ($(findstring realclean, $(MAKECMDGOALS)),)
DONTINCDEP = 1
endif
ifneq  ($(findstring distclean, $(MAKECMDGOALS)),)
DONTINCDEP = 1
endif
ifdef DONTINCDEP
$(info DONTINCDEP set, *.dep files not included)
endif
