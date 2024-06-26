/*
 * Copyright 2008 Sun Microsystems, Inc.  All rights reserved.
 * Use is subject to license terms.
 */


/*
 * lib/krb5/rcache/rcdef.c
 *
 * Copyright 1990 by the Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * Export of this software from the United States of America may
 *   require a specific license from the United States Government.
 *   It is the responsibility of any person or organization contemplating
 *   export to obtain such a license before exporting.
 *
 * WITHIN THAT CONSTRAINT, permission to use, copy, modify, and
 * distribute this software and its documentation for any purpose and
 * without fee is hereby granted, provided that the above copyright
 * notice appear in all copies and that both that copyright notice and
 * this permission notice appear in supporting documentation, and that
 * the name of M.I.T. not be used in advertising or publicity pertaining
 * to distribution of the software without specific, written prior
 * permission.  Furthermore if you modify this software you must label
 * your software as modified software and not distribute it in such a
 * fashion that it might be confused with the original M.I.T. software.
 * M.I.T. makes no representations about the suitability of
 * this software for any purpose.  It is provided "as is" without express
 * or implied warranty.
 *
 *
 * replay cache default operationvectors.
 */

#include "k5-int.h"
#include "rc_file.h"
#include "rc_mem.h"


/*
 * Solaris Kerberos
 * MIT 1.4 just has "dfl" while we now have "FILE" and "MEMORY".
 */
const krb5_rc_ops krb5_rc_file_ops = {
	0,
	"FILE",
	krb5_rc_file_init,
	krb5_rc_file_recover,
	krb5_rc_file_recover_or_init,
	krb5_rc_file_destroy,
	krb5_rc_file_close,
	krb5_rc_file_store,
	krb5_rc_file_expunge,
	krb5_rc_file_get_span,
	krb5_rc_file_get_name,
	krb5_rc_file_resolve
};

const krb5_rc_ops krb5_rc_mem_ops = {
	0,
	"MEMORY",
	krb5_rc_mem_init,
	krb5_rc_mem_recover,
	krb5_rc_mem_recover_or_init,
	krb5_rc_mem_destroy,
	krb5_rc_mem_close,
	krb5_rc_mem_store,
	/* expunging not used in memory rcache type */
	NULL,
	krb5_rc_mem_get_span,
	krb5_rc_mem_get_name,
	krb5_rc_mem_resolve
};

krb5_rc_ops const *krb5_rc_dfl_ops = &krb5_rc_file_ops;
