#define ON  0
#define OFF 1
#define RS  LATEbits.LATE0
#define RW  LATEbits.LATE1
#define E   LATEbits.LATE2
#define _XTAL_FREQ 48000000UL

void POS_CURSOR(unsigned char fila,unsigned char columna);
void DISPLAY_ONOFF(unsigned char estado);
void CURSOR_HOME(void);
void CURSOR_ONOFF(unsigned char estado);
void ENVIA_CHAR(unsigned char dato);
void BORRAR_LCD(void);
void LCD_CONFIG(void);
void ENVIA_NIBBLE(unsigned char dato);
void ENVIA_LCD_CMD(unsigned char dato);
void LEER_LCD(void);
void BLINK_CURSOR(unsigned char val);
void GENERACARACTER(const unsigned char *vector,unsigned char pos);
void ESCRIBE_MENSAJE(const char *cadena,unsigned char tam);