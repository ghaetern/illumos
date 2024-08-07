#
# CDDL HEADER START
#
# The contents of this file are subject to the terms of the
# Common Development and Distribution License, Version 1.0 only
# (the "License").  You may not use this file except in compliance
# with the License.
#
# You can obtain a copy of the license at usr/src/OPENSOLARIS.LICENSE
# or http://www.opensolaris.org/os/licensing.
# See the License for the specific language governing permissions
# and limitations under the License.
#
# When distributing Covered Code, include this CDDL HEADER in each
# file and include the License file at usr/src/OPENSOLARIS.LICENSE.
# If applicable, add the following below this CDDL HEADER, with the
# fields enclosed by brackets "[]" replaced with your own identifying
# information: Portions Copyright [yyyy] [name of copyright owner]
#
# CDDL HEADER END
#
#
# Copyright 2004 Sun Microsystems, Inc.  All rights reserved.
# Use is subject to license terms.
#
# Copyright (c) 2018, Joyent, Inc.

PROG=	talk
OBJS=	talk.o get_names.o display.o io.o ctl.o init_disp.o \
	msgs.o get_addrs.o ctl_transact.o invite.o look_up.o
SRCS= $(OBJS:%.o=%.c)

include ../../../Makefile.cmd

#
# Message catalog
#
POFILE= talk.po
#

CFLAGS +=	$(CCVERBOSE)
CERRWARN +=	$(CNOWARN_UNINIT)
CPPFLAGS +=	-DSYSV -DSTRNET -DBSD_COMP
LDLIBS +=	-lcurses -lsocket -lnsl

# not linted
SMATCH=off

.KEEP_STATE:

.PARALLEL: $(OBJS)

all: $(PROG)

$(PROG): $(OBJS)
	$(LINK.c) $(OBJS) -o $@ $(LDLIBS)
	$(POST_PROCESS)

#
# message catalog
#
_msg: $(POFILE)

$(POFILE): $(SRCS)
	$(RM) talk.po
	$(COMPILE.cpp) 	$(SRCS)	> $(POFILE).i
	$(XGETTEXT) $(XGETFLAGS)	$(POFILE).i
	sed "/^domain/d" messages.po > $@
	$(RM) messages.po $(POFILE).i

#
install: all $(ROOTPROG)

clean:
	$(RM) $(OBJS)

lint:	lint_SRCS

include ../../../Makefile.targ
