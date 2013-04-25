#include "misc.h"

#ifdef CONFIG_RANDOMIZE_BASE

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

	if (cmdline_find_option_bool("nokaslr")) {
		debug_putstr("KASLR disabled...\n");
		goto out;
	}

	choice = find_minimum_location((unsigned long)input, input_size,
				       (unsigned long)output, output_size);

	/* XXX: choose random location. */

out:
	return (unsigned char *)choice;
}

#endif /* CONFIG_RANDOMIZE_BASE */
