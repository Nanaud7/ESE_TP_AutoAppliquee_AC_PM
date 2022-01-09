/**
 ******************************************************************************
 * @file	myShell.c
 * @author 	Arnaud CHOBERT (base de code fournie par Laurent FIACK)
 * @brief	Shell pour STM32
 ******************************************************************************
 */

#include "myShell.h"

/* Types ---------------------------------------------------------------------*/

// Structure pour une commande du Shell
typedef struct{
	char * cmd;
	int (* func)(int argc, char ** argv);
	char * description;
} shell_func_t;

/* End of types --------------------------------------------------------------*/

/* Macros --------------------------------------------------------------------*/
#define SHELL_FUNC_LIST_MAX_SIZE 16
#define SHELL_CMD_MAX_SIZE 16
#define SHELL_ARGC_MAX 8
#define SHELL_BUFFER_SIZE 40
/* End of macros -------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
UART_HandleTypeDef* shell_huart = NULL;
char starting[] = "\r\n\r\n===== Shell =====\r\n";
char prompt[] = "@Nucleo-G431 >> ";

char c = 0; // Caractère reçu
uint8_t pos = 0; // Position dans le buffer
static char buf[SHELL_BUFFER_SIZE];
static char backspace[] = "\b \b";
uint8_t it_uart_rx_ready = 0;

static int shell_func_list_size = 0;
static shell_func_t shell_func_list[SHELL_FUNC_LIST_MAX_SIZE];

/* End of variables ----------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**
 * __io_putchar : Indispensable pour printf() sur la liaison uart
 * @param ch Caractère à écrire sur la liaison uart
 * @return 0 si HAL_OK
 */
uint8_t __io_putchar(int ch) {
	if(HAL_OK != HAL_UART_Transmit(shell_huart, (uint8_t *)&ch, 1, HAL_MAX_DELAY)){
		return 1;
	}
	return 0;
}

/**
 * uart_write : Ecrire sur la liaison uart
 * @param s Chaîne de caractères à écrire sur la liaison uart
 * @param size Longueur de la chaîne de caractère
 * @return 0 si HAL_OK
 */
uint8_t uart_write(char *s, uint16_t size) {
	if(HAL_OK != HAL_UART_Transmit(shell_huart, (uint8_t*)s, size, 0xFFFF)){
		return 1;
	}
	return 0;
}

/**
 * sh_help : Menu d'aide du Shell
 * @param argc Nombre d'éléments de la ligne de commande
 * @param argv Eléments sous la forme d'un tableau de chaînes de caractères
 * @return 0
 */
uint8_t sh_help(int argc, char ** argv) {
	int i;
	for(i = 0 ; i < shell_func_list_size ; i++) {
		printf("%s : %s\r\n", shell_func_list[i].cmd, shell_func_list[i].description);
	}
	return 0;
}

/**
 * sh_example : Fonction d'exemple pour le Shell
 * @param argc Nombre d'éléments de la ligne de commande
 * @param argv Eléments sous la forme d'un tableau de chaînes de caractères
 * @return 0
 */
uint8_t sh_example(int argc, char ** argv) {
	printf("argc = %d\r\n", argc);
	for (int i = 0 ; i < argc ; i++) {
		printf("arg numero %d = %s\r\n", i, argv[i]);
	}
	return 0;
}

/**
 *shell_init : Initialisation du Shell
 * @param huart UART Handle à utiliser
 * @return 0 si HAL_OK
 */
uint8_t shell_init(UART_HandleTypeDef* huart) {
	shell_huart = huart;

	uart_write(starting,sizeof(starting));
	uart_write(prompt,sizeof(prompt));

	shell_add("help", sh_help, (char *)"help");
	shell_add("f", sh_example, "Commande exemple");

	if(HAL_OK != HAL_UART_Receive_IT(shell_huart, (uint8_t*)&c, 1)){
		return 1;
	}

	return 0;
}

/**
 * shell_add : Ajout d'une commande
 * @param cmd Commande à rentrer
 * @param pfunc Référence vers la fonction à appeler
 * @param description Description de la fonction pour le menu d'aide
 * @return 0 si le nombre de commande est valide
 */
uint8_t shell_add(char * cmd, int (* pfunc)(int argc, char ** argv), char * description) {
	if (shell_func_list_size < SHELL_FUNC_LIST_MAX_SIZE) {
		shell_func_list[shell_func_list_size].cmd = cmd;
		shell_func_list[shell_func_list_size].func = pfunc;
		shell_func_list[shell_func_list_size].description = description;
		shell_func_list_size++;
		return 0;
	}

	return -1;
}

/**
 * shell_char_received : Traitement du dernier caractère reçu
 * @return 0
 */
uint8_t shell_char_received() {

	switch (c) {
	case '\r':
		// Appui sur la touche ENTER
		printf("\r\n");
		buf[pos++] = 0;
		pos = 0;
		shell_exec(buf);
		uart_write(prompt,sizeof(prompt));
		break;

	case '\b':
		// Appui sur la touche DELETE
		if (pos > 0) {
			pos--;
			uart_write(backspace, 3);
		}
		break;

	default:
		if (pos < SHELL_BUFFER_SIZE) {
			uart_write(&c, 1);
			buf[pos++] = c;
		}
	}

	return 0;
}

/**
 * shell_exec : Recherche et execution de la commande
 * @param cmd Commande à traiter
 * @return Référence vers la fonction à exécuter
 */
uint8_t shell_exec(char * cmd) {
	int argc;
	char * argv[SHELL_ARGC_MAX];
	char *p;

	// Séparation du header et des paramètres
	char header[SHELL_CMD_MAX_SIZE] = "";
	int h = 0;

	while(cmd[h] != ' ' && h < SHELL_CMD_MAX_SIZE){
		header[h] = cmd[h];
		h++;
	}
	header[h] = '\0';

	// Recherche de la commande et paramètres
	for(int i = 0 ; i < shell_func_list_size ; i++) {
		if (!strcmp(shell_func_list[i].cmd, header)) {
			argc = 1;
			argv[0] = cmd;

			for(p = cmd ; *p != '\0' && argc < SHELL_ARGC_MAX ; p++){
				if(*p == ' ') {
					*p = '\0';
					argv[argc++] = p+1;
				}
			}

			return shell_func_list[i].func(argc, argv);
		}
	}
	printf("%s: command not found\r\n", cmd);
	return -1;
}

/* End of functions ----------------------------------------------------------*/
