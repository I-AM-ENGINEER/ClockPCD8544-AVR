// Такая частота выбрана потому, что хорошо подходит для синхронизации на стандартных скоростях UART
#define F_CPU 7372800UL 

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <util/delay.h>
#include <time.h>
#include <stdbool.h>

#include "font.h"

#define LCD_PORT		PORTB
#define LCD_DC_PIN		PB4
#define LCD_RST_PIN		PB5
#define LCD_CE_PIN		PB6

#define _LCD_DC_SET             	LCD_PORT |=  (1<<LCD_DC_PIN)
#define _LCD_DC_RESET             	LCD_PORT &= ~(1<<LCD_DC_PIN)
#define _LCD_CE_SET             	LCD_PORT |=  (1<<LCD_CE_PIN)
#define _LCD_CE_RESET              	LCD_PORT &= ~(1<<LCD_CE_PIN)
#define _LCD_RST_SET            	LCD_PORT |=  (1<<LCD_RST_PIN)
#define _LCD_RST_RESET             	LCD_PORT &= ~(1<<LCD_RST_PIN)

#define BTN_PINX		PINC
#define BTN_UP_PIN		PC0
#define BTN_OK_PIN		PC1
#define BTN_DN_PIN		PC2

typedef enum {
	btn_state_pressed,
	btn_state_unpressed,
} btn_state_t;

typedef enum {
	cursor_none,
	cursor_hr_h,
	cursor_hr_l,
	cursor_mn_h,
	cursor_mn_l,
	cursor_s_h,
	cursor_s_l,
	cursor_en,
} alarm_cursor_select_t;

volatile alarm_cursor_select_t alarm_cursor_position;

btn_state_t btn_up_state = btn_state_unpressed;
btn_state_t btn_ok_state = btn_state_unpressed;
btn_state_t btn_dn_state = btn_state_unpressed;

uint8_t lcd_cursor_x;
uint8_t lcd_cursor_y;

volatile time_t alarm_time_set;
time_t alarm_time;
volatile bool alarm = false;
volatile bool alarm_en = false;

char usart_rx_buffer[16];

void usart_send_byte( uint8_t data ){
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void usart_rx_complete_irq( char *buffer ){
	set_system_time((time_t)atol(buffer));
}

void cursor_update_value(bool dir){
	// Знаем позицию курсора - знаем и на сколько менять время
	int8_t i = dir ? 1 : -1;
	switch (alarm_cursor_position){
		case cursor_hr_l:	alarm_time_set += 3600 * i; 	break;
		case cursor_hr_h: 	alarm_time_set += 36000 * i; 	break;
		case cursor_mn_l: 	alarm_time_set += 60 * i; 		break;
		case cursor_mn_h: 	alarm_time_set += 600 * i; 		break;
		case cursor_s_l: 	alarm_time_set += i; 			break;
		case cursor_s_h: 	alarm_time_set += 10 * i; 		break;
		case cursor_en:		alarm_en = !alarm_en;		break;
		default: break;
	}
	// Если улетели за 24 часа, возвращаемся в 0
	if(alarm_time_set > 86400){
		alarm_time_set = 0;
	}
}

void cursor_next(bool dir){
	if(alarm_cursor_position == cursor_en){
		alarm_cursor_position = cursor_none;
	}else{
		alarm_cursor_position++;
	}
}

void spi_write_byte( uint8_t byte ){
	SPDR = byte; // Пишем байт
	while(!(SPSR & (1<<SPIF))); // Ждем завершения передачи
}

void lcd_write_cmd(uint8_t cmd){
	_LCD_DC_RESET;
	_LCD_CE_RESET;
	spi_write_byte(cmd);
	_LCD_CE_SET;
}

void lcd_write_data(uint8_t cmd){
	_LCD_DC_SET;
	_LCD_CE_RESET;
	spi_write_byte(cmd);
	_LCD_CE_SET;
}

void lcd_set_cursor(uint8_t x, uint8_t y){
	lcd_cursor_x = x;
	lcd_cursor_y = y;
	lcd_write_cmd(0x80 | x);  // Столбец
    lcd_write_cmd(0x40 | y);  // Строка
}

void lcd_putc( char c ){
	c -= 0x20;
	for(uint8_t i = 0; i < 5; i++){
		// Установка курсора, там где должен быть столбец части символа
		lcd_set_cursor(lcd_cursor_x, lcd_cursor_y);
		// Вывод части символа
		lcd_write_data(pgm_read_byte(&font_5x8[(uint8_t)c][i]));
		lcd_cursor_x++;
	}
	lcd_write_data(0x00);
}

void lcd_putc_inv( char c ){
	// То же самое, но с инверсией
	c -= 0x20;
	for(uint8_t i = 0; i < 5; i++){
		lcd_set_cursor(lcd_cursor_x, lcd_cursor_y);
		lcd_write_data(~pgm_read_byte(&font_5x8[(uint8_t)c][i]));
		lcd_cursor_x++;
	}
	lcd_write_data(0xFF);
}

void lcd_puts( const char* str, bool inv ){
	while(*str){
		if(inv){
			lcd_putc_inv(*str);
		}else{
			lcd_putc(*str);
		}
		lcd_cursor_x++;
		str++;
	}
}

void lcd_init( void ){
	_LCD_DC_RESET;
	_LCD_RST_RESET;
	_delay_ms(50);
	_LCD_RST_SET;
	_delay_ms(10);
	// копипаст с даташита
	lcd_write_cmd(0x21);
	lcd_write_cmd(0x13);
	lcd_write_cmd(0x06);
	lcd_write_cmd(0xC2);
	lcd_write_cmd(0x20);
	lcd_write_cmd(0x09);
	lcd_write_cmd(0x80);
	lcd_write_cmd(0x40);
	// Переход в нормальный режим работы
	lcd_write_cmd(0x08);
	lcd_write_cmd(0x0C);
}

int main( void ){
	// Инициализация UART
	// При UBRR=47, 9600 бод при FCPU=7.372МГц
	uint16_t ubrr = 47;
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)ubrr;
	// Включение приема, передачи, прерывания при приеме байта
	UCSR0B = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE0);
	// Длина пакета 8 бит
	UCSR0C = (3<<UCSZ0);

	// Настройка часового таймера (TIM0)
	// Включен асинхронный режим таймера 0, так таймер тактируется от внешнего кварцевого резонатора
	// с частотой  32768 (часовой кварц) для обеспечения точного хода времени
	ASSR = (1<<AS0);
	// Делитель 128, нормальный режим работы
	// При делителе 128, таймер переполняется ровно 1 раз в секунду
	TCCR0 = (1<<CS02)|(1<<CS00); 
	// Включить прерывание по переполнению
	TIMSK = (1<<TOIE0);

	// Частота переполнения 112,5 Гц - таймер для опроса кнопок
	TCCR1B = (1<<CS10); // Тактирование без делителя
	TIMSK |= (1<<TOIE1); // Включить прерывание по переполнению

	// Настройка SPI для дисплея
	DDRB = (1<<PB1)|(1<<PB2)|(1<<PB0); // MOSI, SCK в режим выхода
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);// Включить SPI, режим мастера, делитель 64

	// Инициализация дисплея
	// Нужные пины в режим выхода
	DDRB |= (1<<LCD_DC_PIN)|(1<<LCD_RST_PIN)|(1<<LCD_CE_PIN);
	lcd_init();

	PORTC = (1<<BTN_UP_PIN)|(1<<BTN_OK_PIN)|(1<<BTN_DN_PIN);

	sei(); // Глобальные прерывания включить

	char alarm_time_str[32];
	
	while (1){
		time_t current_time = time(NULL); // Узнаем текущее время
		// Преобразуем в человеческий вид
		struct tm *current_day_tm = localtime(&current_time); 
		// Надо убрать часы/минуты/секунды, для того, что бы узнать время начала дня
		current_day_tm->tm_hour = 0;
		current_day_tm->tm_min = 0;
		current_day_tm->tm_sec = 0;
		// Перевод обратно в timestamp
		time_t current_day_time = mktime(current_day_tm);
		// Узнаем таймстамп будильника
		alarm_time = current_day_time + alarm_time_set;
		// Если будильник прошел, считаем, что он будет завтра
		if(difftime(alarm_time, current_time) < 0){
			alarm_time += 86400;
		}
		// Перевод времени в читаемый человеком вид
		char* date_str = ctime(&current_time);
		char* time_str = date_str;
		
		
		time_str = strchr(time_str, ' ') + 1;
		time_str = strchr(time_str, ' ') + 1;
		time_str = strchr(time_str, ' ') + 1;
		time_str[-1] = '\0';

		// Вывод текущего времени и даты
		lcd_set_cursor(0,0);
		lcd_puts("Current time", false);
		lcd_set_cursor(0,9);
		lcd_puts(time_str, false);
		lcd_set_cursor(0,18);
		lcd_puts(date_str, false);
		lcd_set_cursor(0,27);
		lcd_puts("Alarm set", false);
		
		// Вывод времени будильника
		struct tm *alarm_time_tm = localtime(&alarm_time);
		lcd_set_cursor(0,36);
		sprintf(alarm_time_str, "%02d:%02d:%02d ", alarm_time_tm->tm_hour, alarm_time_tm->tm_min, alarm_time_tm->tm_sec);
		// Эм... Говнокод для того, что бы понять, где должен быть курсор
		for(uint8_t i = 0; i < 9; i+=3){
			for(uint8_t j = 0; j < 2; j++){
				if(alarm_cursor_position == (i+j - (i/3) + 1)){
					lcd_putc_inv(alarm_time_str[i+j]);
				}else{
					lcd_putc(alarm_time_str[i+j]);
				}
			}
			lcd_putc(alarm_time_str[i+2]);
		}
		// Будильник включен?
		bool select_en = alarm_cursor_position == 7;
		if(alarm_en){
			lcd_puts("ON ", select_en);
		}else{
			lcd_puts("OFF", select_en);
		}
		// Алярма
		lcd_set_cursor(2,45);
		if(alarm){
			lcd_puts("   !ALARM!   ", true);
		}else{
			lcd_puts("             ", false);
		}
		// Что бы не обновлять дисплей постоянно
		_delay_ms(50);
	}
}

ISR(USART0_RX_vect){ // Прерывание при приеме байта по UART
	static uint8_t usart_rx_len;
	uint8_t received_byte = UDR0;
	usart_send_byte(received_byte);
	usart_rx_buffer[usart_rx_len] = received_byte;
	usart_rx_len++;
	// Проверяем на терминирующий символ, или выход за пределы стека
	if(	(received_byte == '\0') ||\
		(received_byte == '\r') ||\
		(received_byte == '\n') ||\
		((usart_rx_len+1) == sizeof(usart_rx_buffer))
	){
		usart_rx_complete_irq(usart_rx_buffer);
		usart_rx_len = 0;
	}
}

ISR(TIMER0_OVF_vect){ // Прерывание часового таймера
	// Проверяем, нужен ли аларм
	system_tick();
	if((time(NULL) == alarm_time) && alarm_en){
		alarm = true;
	}
}

ISR(TIMER1_OVF_vect){ // Прерывание таймера опроса кнопок
	if((btn_ok_state == btn_state_pressed) && (BTN_PINX & (1 << BTN_OK_PIN))){
		btn_ok_state = btn_state_unpressed;
	}else if((btn_ok_state == btn_state_unpressed) && !(BTN_PINX & (1 << BTN_OK_PIN))){
		btn_ok_state = btn_state_pressed;
		// Если нажата кнопка ОК
		if(alarm){ // сброс алярма
			alarm = false;
			return;
		}
		// Сдвигаем курсор
		cursor_next(true);
	}

	if((btn_up_state == btn_state_pressed) && (BTN_PINX & (1 << BTN_UP_PIN))){
		btn_up_state = btn_state_unpressed;
	}else if((btn_up_state == btn_state_unpressed) && !(BTN_PINX & (1 << BTN_UP_PIN))){
		btn_up_state = btn_state_pressed;
		// Если нажата кнопка вверх
		cursor_update_value(true);
	}

	if((btn_dn_state == btn_state_pressed) && (BTN_PINX & (1 << BTN_DN_PIN))){
		btn_dn_state = btn_state_unpressed;
	}else if((btn_dn_state == btn_state_unpressed) && !(BTN_PINX & (1 << BTN_DN_PIN))){
		btn_dn_state = btn_state_pressed;
		// Если нажата кнопка вниз
		cursor_update_value(false);
	}
}
