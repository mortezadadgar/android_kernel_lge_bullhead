#include "misc.h"

#ifdef CONFIG_RANDOMIZE_BASE
#include <asm/msr.h>
#include <asm/e820.h>

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

/*
 * This routine is used to count how many aligned slots that can hold the
 * kernel exist across all e820 entries. It is called in two phases, once
 * to count valid memory regions available for the kernel image, and a
 * second time to select one from those seen.
 *
 * It is first called with "counting" set to true, where it expects to be
 * called once for each e820 entry. In this mode, it will update *count
 * with how many slots are available in the given e820 entry. Once the walk
 * across all e820 entries has finished, the caller will have a total count
 * of all valid memory regions available for the kernel image.
 *
 * Once the first pass of entry walking is finished, the caller selects one
 * of the possible slots (stored in *count), and performs a second walk,
 * with "counting" set to false. In this mode, *count is decremented until
 * the corresponding slot is found in a specific e820 region, at which
 * point, the function returns that address, and the walk terminates.
 */
static unsigned long process_e820_entry(struct e820entry *entry, bool counting,
					unsigned long minimum,
					unsigned long image_size,
					unsigned long *count)
{
	u64 addr, size;
	unsigned long alignments;

	/* Skip non-RAM entries. */
	if (entry->type != E820_RAM)
		return 0;

	/* Ignore entries entirely above our maximum. */
	if (entry->addr >= CONFIG_RANDOMIZE_BASE_MAX_OFFSET)
		return 0;

	/* Ignore entries entirely below our minimum. */
	if (entry->addr + entry->size < minimum)
		return 0;

	size = entry->size;
	addr = entry->addr;

	/* Potentially raise address to minimum location. */
	if (addr < minimum)
		addr = minimum;

	/* Potentially raise address to meet alignment requirements. */
	addr = ALIGN(addr, CONFIG_PHYSICAL_ALIGN);

	/* Did we raise the address above the bounds of this e820 region? */
	if (addr > entry->addr + entry->size)
		return 0;

	/* Reduce size by any delta from the original address. */
	size -= addr - entry->addr;

	/* Reduce maximum image starting location to maximum limit. */
	if (addr + size > CONFIG_RANDOMIZE_BASE_MAX_OFFSET)
		size = CONFIG_RANDOMIZE_BASE_MAX_OFFSET - addr;

	/* Ignore entries that cannot hold even a single kernel image. */
	if (size < image_size)
		return 0;

	/*
	 * Reduce size by kernel image size so we can see how many aligned
	 * starting addresses will fit without running past the end of a
	 * region. XXX: adjacent e820 regions are not detected, so up to
	 * image_size / CONFIG_PHYSICAL_ALIGN slots may go unused across
	 * adjacent regions.
	 */
	size -= image_size;

	/* Now we know how many aligned slots can contain the image. */
	alignments = (size / CONFIG_PHYSICAL_ALIGN) + 1;

	/* In the first pass, just counting all the e820 entries? */
	if (counting) {
		*count += alignments;
		return 0;
	}

	/* Otherwise we're counting down to find a specific aligned slot. */
	if (*count < alignments) {
		/* Desired region is in this entry. */
		return addr + (CONFIG_PHYSICAL_ALIGN * *count);
	} else {
		/* Desired region is beyond this entry. */
		*count -= alignments;
		return 0;
	}
}

static unsigned long find_random_e820(unsigned long minimum,
				      unsigned long size)
{
	int i;
	unsigned long addr, count;

	/* Make sure minimum is aligned. */
	minimum = ALIGN(minimum, CONFIG_PHYSICAL_ALIGN);

	/* Verify potential e820 positions. */
	count = 0;
	for (i = 0; i < real_mode->e820_entries; i++) {
		process_e820_entry(&real_mode->e820_map[i], true, minimum,
				   size, &count);
	}

	/* Handle crazy case of nothing fitting. */
	if (count == 0)
		return 0;

	count = get_random_long() % count;

	/* Select desired e820 position. */
	for (i = 0; i < real_mode->e820_entries; i++) {
		addr = process_e820_entry(&real_mode->e820_map[i], false,
					  minimum, size, &count);
		if (addr)
			return addr;
	}
	return 0;
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

	random = find_random_e820(choice, output_size);
	if (!random) {
		debug_putstr("KASLR could not find suitable E820 region...\n");
		goto out;
	}

	/* Always enforce the minimum. */
	if (random < choice)
		goto out;

	choice = random;
out:
	return (unsigned char *)choice;
}

#endif /* CONFIG_RANDOMIZE_BASE */
