
#include <common.h>
#include <config.h>
#include <command.h>
#include <asm/arch/gpio.h>

enum antena_cmd { ANTENA_ON, ANTENA_OFF };

enum antena_cmd get_antena_cmd(char *var)
{
	if (strcmp(var, "0") == 0) {
		return ANTENA_OFF;
	}
	if (strcmp(var, "1") == 0) {
		return ANTENA_ON;
	}
	return -1;
}

int do_antena (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	enum antena_cmd cmd;

	/* Validate arguments */
	if ((argc != 2)) {
		return CMD_RET_USAGE;
	}

	cmd = get_antena_cmd(argv[1]);
	if (cmd < 0) {
		return CMD_RET_USAGE;
	}
	switch (cmd) {
	case ANTENA_ON:
		
		at91_set_pio_output(AT91_PIO_PORTC, 9, 1);
		break;
	case ANTENA_OFF:
		at91_set_pio_output(AT91_PIO_PORTC, 9, 0);
		break;
	}
	
	return 0;
}

/*

*/
U_BOOT_CMD(
	antena, 3, 1, do_antena,
	"Switch antena pin on or off",
	"[1/0]"
);
