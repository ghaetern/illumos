/
/ Copyright 2004 Sun Microsystems, Inc.  All rights reserved.
/ Use is subject to license terms.
/

/ XX64 - fix me - this is a copy of the x86 version

        .ident	"%Z%%M%	%I%	%E% SMI"

/ -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 2 -*-
/
/ The contents of this file are subject to the Mozilla Public
/ License Version 1.1 (the "License"); you may not use this file
/ except in compliance with the License. You may obtain a copy of
/ the License at http://www.mozilla.org/MPL/
/
/ Software distributed under the License is distributed on an "AS
/ IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
/ implied. See the License for the specific language governing
/ rights and limitations under the License.
/
/ The Original Code is the Netscape Portable Runtime (NSPR).
/
/ The Initial Developer of the Original Code is Netscape
/ Communications Corporation.  Portions created by Netscape are
/ Copyright (C) 1998-2000 Netscape Communications Corporation.  All
/ Rights Reserved.
/
/ Contributor(s):
/
/ Alternatively, the contents of this file may be used under the
/ terms of the GNU General Public License Version 2 or later (the
/ "GPL"), in which case the provisions of the GPL are applicable
/ instead of those above.  If you wish to allow use of your
/ version of this file only under the terms of the GPL and not to
/ allow others to use your version of this file under the MPL,
/ indicate your decision by deleting the provisions above and
/ replace them with the notice and other provisions required by
/ the GPL.  If you do not delete the provisions above, a recipient
/ may use your version of this file under either the MPL or the
/ GPL.
/

	.text

	.globl	getedi
getedi:
	movl	%edi,%eax
	ret
	.type	getedi,@function
	.size	getedi,.-getedi

	.globl	setedi
setedi:
	movl	4(%rsp),%edi
	ret
	.type	setedi,@function
	.size	setedi,.-setedi

	.globl	__MD_FlushRegisterWindows
	.globl _MD_FlushRegisterWindows

__MD_FlushRegisterWindows:
_MD_FlushRegisterWindows:

	ret

/
/ sol_getsp()
/
/ Return the current sp (for debugging)
/
	.globl sol_getsp
sol_getsp:
	movl	%esp, %eax
	ret

/
/ sol_curthread()
/
/ Return a unique identifier for the currently active thread.
/
	.globl sol_curthread
sol_curthread:
	movl	%ecx, %eax
	ret

/ PRInt32 _MD_AtomicIncrement(PRInt32 *val)
/
/ Atomically increment the integer pointed to by 'val' and return
/ the result of the increment.
/
    .text
    .globl _MD_AtomicIncrement
    .align 4
_MD_AtomicIncrement:
    movl 4(%rsp), %ecx
    movl $1, %eax
    lock
    xaddl %eax, (%rcx)
    incl %eax
    ret

/ PRInt32 _MD_AtomicDecrement(PRInt32 *val)
/
/ Atomically decrement the integer pointed to by 'val' and return
/ the result of the decrement.
/
    .text
    .globl _MD_AtomicDecrement
    .align 4
_MD_AtomicDecrement:
    movl 4(%rsp), %ecx
    movl $-1, %eax
    lock
    xaddl %eax, (%rcx)
    decl %eax
    ret

/ PRInt32 _MD_AtomicSet(PRInt32 *val, PRInt32 newval)
/
/ Atomically set the integer pointed to by 'val' to the new
/ value 'newval' and return the old value.
/
/ An alternative implementation:
/   .text
/   .globl _MD_AtomicSet
/   .align 4
/_MD_AtomicSet:
/   movl 4(%rsp), %ecx
/   movl 8(%rsp), %edx
/   movl (%rcx), %eax
/retry:
/   lock
/   cmpxchgl %edx, (%rcx)
/   jne retry
/   ret
/
    .text
    .globl _MD_AtomicSet
    .align 4
_MD_AtomicSet:
    movl 4(%rsp), %ecx
    movl 8(%rsp), %eax
    lock
    xchgl %eax, (%rcx)
    ret

/ PRInt32 _MD_AtomicAdd(PRInt32 *ptr, PRInt32 val)
/
/ Atomically add 'val' to the integer pointed to by 'ptr'
/ and return the result of the addition.
/
    .text
    .globl _MD_AtomicAdd
    .align 4
_MD_AtomicAdd:
    movl 4(%rsp), %ecx
    movl 8(%rsp), %eax
    movl %eax, %edx
    lock
    xaddl %eax, (%rcx)
    addl %edx, %eax
    ret

/
/ PR_StackPush(listp, elementp)
/
/ Atomically push ElementP onto linked list ListP.
/
	.text
	.globl	PR_StackPush
	.align	4
PR_StackPush:
	movl	4(%rsp), %ecx
	movl	$-1,%eax
pulock:
/ Already locked?
	cmpl	%eax,(%rcx)
	je	pulock

/ Attempt to lock it
	lock
	xchgl	%eax, (%rcx)

/ Did we set the lock?
	cmpl	$-1, %eax
	je	pulock

/ We now have the lock.  Update pointers
	movl	8(%rsp), %edx
	movl	%eax, (%rdx)
	movl	%edx, (%rcx)

/ Done
	ret


/
/ elementp = PR_StackPop(listp)
/
/ Atomically pop ElementP off linked list ListP
/
	.text
	.globl	PR_StackPop
	.align	4
PR_StackPop:
	movl	4(%rsp), %ecx
	movl	$-1, %eax
polock:
/ Already locked?
	cmpl	%eax, (%rcx)
	je	polock

/ Attempt to lock it
	lock
	xchgl	%eax, (%rcx)

/ Did we set the lock?
	cmpl	$-1, %eax
	je	polock

/ We set the lock so now update pointers

/ Was it empty?
	movl	$0, %edx
	cmpl	%eax,%edx
	je	empty

/ Get element "next" pointer
	movl	(%rax), %edx

/ Write NULL to the element "next" pointer
	movl	$0, (%rax)

empty:
/ Put elements previous "next" value into listp
/ NOTE: This also unlocks the listp
	movl	%edx, (%rcx)

/ Return previous listp value (already in eax)
	ret
