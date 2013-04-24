#include "misc.h"

#ifdef CONFIG_RANDOMIZE_BASE
#include <asm/msr.h>

#include <asm/archrandom.h>
static inline int rdrand(unsigned long *v)
{
	int ok;
	asm volatile("1: " RDRAND_LONG "\n\t"
		     "jc 2f\n\t"
		     "decl %0\n\t"
		     "jnz 1b\n\t"
		     "2:"
		     : "=r" (ok), "=a" (*v)
		     : "0" (RDRAND_RETRY_LOOPS));
	return ok;
}

#define I8254_PORT_CONTROL	0x43
#define I8254_PORT_COUNTER0	0x40
#define I8254_CMD_READBACK	0xC0
#define I8254_SELECT_COUNTER0	0x02
#define I8254_STATUS_NOTREADY	0x40
static inline u16 i8254(void)
{
	u16 status, timer;

	do {
		outb(I8254_PORT_CONTROL,
		     I8254_CMD_READBACK | I8254_SELECT_COUNTER0);
		status = inb(I8254_PORT_COUNTER0);
		timer  = inb(I8254_PORT_COUNTER0);
		timer |= inb(I8254_PORT_COUNTER0) << 8;
	} while (status & I8254_STATUS_NOTREADY);

	return timer;
}

static unsigned long get_random_long(void)
{
	unsigned long random;

	if (has_cpuflag(X86_FEATURE_RDRAND)) {
		debug_putstr("KASLR using RDRAND...\n");
		if (rdrand(&random))
			return random;
	}

	if (has_cpuflag(X86_FEATURE_TSC)) {
		uint32_t raw;

		debug_putstr("KASLR using RDTSC...\n");
		rdtscl(raw);

		/* Only use the low bits of rdtsc. */
		random = raw & 0xffff;
	} else {
		debug_putstr("KASLR using i8254...\n");
		random = i8254();
	}

	/* Extend timer bits poorly... */
	random |= (random << 16);
#ifdef CONFIG_X86_64
	random |= (random << 32) | (random << 48);
#endif
	return random;
}

static unsigned long find_minimum_location(unsigned long input,
					   unsigned long input_size,
					   unsigned long output,
					   unsigned long output_size)
{
	u64 initrd_start, initrd_size;
	u64 cmd_line, cmd_line_size;
	unsigned long unsafe, unsafe_len;
	char *ptr;

	/*
	 * Mark off the region that is unsafe to overlap during
	 * decompression (see calculations at top of misc.c).
	 */
	unsafe_len = (output_size >> 12) + 32768 + 18;
	unsafe = (unsigned long)input + input_size - unsafe_len;

	/*
	 * Locate other regions that cannot be over-written during
	 * decompression: initrd, cmd_line.
	 */
	initrd_start  = (u64)real_mode->ext_ramdisk_image << 32;
	initrd_start |= real_mode->hdr.ramdisk_image;
	initrd_size  = (u64)real_mode->ext_ramdisk_size << 32;
	initrd_size |= real_mode->hdr.ramdisk_size;
	cmd_line  = (u64)real_mode->ext_cmd_line_ptr << 32;
	cmd_line |= real_mode->hdr.cmd_line_ptr;
	/* Calculate size of cmd_line. */
	ptr = (char *)(unsigned long)cmd_line;
	for (cmd_line_size = 0; ptr[cmd_line_size++]; )
		;

	/* Minimum location must be above all these regions: */
	output = max(output, unsafe + unsafe_len);
	output = max(output, (unsigned long)free_mem_ptr + BOOT_HEAP_SIZE);
	output = max(output, (unsigned long)free_mem_end_ptr + BOOT_STACK_SIZE);
	output = max(output, (unsigned long)initrd_start
			     + (unsigned long)initrd_size);
	output = max(output, (unsigned long)cmd_line
			     + (unsigned long)cmd_line_size);

	/* Make sure the location is still aligned. */
	output = ALIGN(output, CONFIG_PHYSICAL_ALIGN);

	return output;
}

unsigned char *choose_kernel_location(unsigned char *input,
				      unsigned long input_size,
				      unsigned char *output,
				      unsigned long output_size)
{
	unsigned long choice = (unsigned long)output;
	unsigned long random;

	if (cmdline_find_option_bool("nokaslr")) {
		debug_putstr("KASLR disabled...\n");
		goto out;
	}

	choice = find_minimum_location((unsigned long)input, input_size,
				       (unsigned long)output, output_size);

	/* XXX: Find an appropriate E820 hole, instead of adding offset. */
	random = get_random_long();

	/* Clip off top of the range. */
	random &= (CONFIG_RANDOMIZE_BASE_MAX_OFFSET - 1);
	while (random + output_size > CONFIG_RANDOMIZE_BASE_MAX_OFFSET)
		random >>= 1;

	/* Clip off bottom of range. */
	random &= ~(choice - 1);

	/* Always enforce the minimum. */
	if (random < choice)
		goto out;

	choice = random;
out:
	return (unsigned char *)choice;
}

#endif /* CONFIG_RANDOMIZE_BASE */
