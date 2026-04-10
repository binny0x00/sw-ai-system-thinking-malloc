	.arch armv8-a
	.file	"process_memory_layout.c"
	.text
	.global	global_init
	.data
	.align	2
	.type	global_init, %object
	.size	global_init, 4
global_init:
	.word	10
	.global	global_uninit
	.bss
	.align	2
	.type	global_uninit, %object
	.size	global_uninit, 4
global_uninit:
	.zero	4
	.section	.rodata
	.align	3
.LC0:
	.string	"%d\n"
	.text
	.align	2
	.global	main
	.type	main, %function
main:
.LFB0:
	.cfi_startproc
	stp	x29, x30, [sp, -32]!
	.cfi_def_cfa_offset 32
	.cfi_offset 29, -32
	.cfi_offset 30, -24
	mov	x29, sp
	mov	w0, 5
	str	w0, [sp, 28]
	adrp	x0, global_init
	add	x0, x0, :lo12:global_init
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, .LC0
	add	x0, x0, :lo12:.LC0
	bl	printf
	mov	w0, 0
	ldp	x29, x30, [sp], 32
	.cfi_restore 30
	.cfi_restore 29
	.cfi_def_cfa_offset 0
	ret
	.cfi_endproc
.LFE0:
	.size	main, .-main
	.data
	.align	2
	.type	static_var.0, %object
	.size	static_var.0, 4
static_var.0:
	.word	20
	.ident	"GCC: (Ubuntu 13.3.0-6ubuntu2~24.04.1) 13.3.0"
	.section	.note.GNU-stack,"",@progbits
