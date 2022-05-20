/* Emit some NOPs to .ctors to work around a bootloader issue on Model 01 */
#ifdef EMIT_NOPS
	.section .ctors
	.skip 16
#endif
