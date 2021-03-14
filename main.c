// --------- Agnieszka Trendowicz ------- //
// --------- Elektronika w Medycynie ---- //
// --------- Systemy wbudowane ---------- //
// --------- Projekt nr 2 --------------- //


// POLACZENIA
 
// CZUJNIK:
// TRIGGER -> PD2			
// ECHO -> (INT1) PD3
// LED -> PD5 (OC1A)

// USART: po³¹czenie krzy¿owe
// TxD -> PD0 (RxD)
// RxD -> PD1 (TxD)

#define F_CPU 8000000UL					// 8MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>

#include "uart.h"						// biblioteki USART
#define UART_BAUD_RATE 9600				


#define LED_PIN (1<<PD5)				// pomocniczo

void Wyslij_sygnal(void);				// odpowiada za aktywacjê triggera
void wlacz_przerwania (void);			// przerwania od timera 0
void wlacz_timer0 (void);
void wlacz_uart (void);					// uart bez przerwania, funkcja transmittera, baud 9600

unsigned char praca; 					// sensor jest zajêty - pomiar, sensor wolny - wynik otrzymany
unsigned char rising_edge;
volatile uint16_t czas_pomiaru;			// liczymy czas trwania echa
volatile int distance_cm;				// tu zachowujemy wartoœæ w cm
volatile uint8_t error;




// przerwania w timerze 0
ISR (TIMER0_OVF_vect)
{
	if(rising_edge==1) 					// czy jest echo?
	{
		czas_pomiaru++;					// pomiar

		if(czas_pomiaru > 91)
		{
			praca = 0;
			rising_edge = 0;
			error = 1;
		}
	}
}

ISR (INT1_vect)
{
	if(praca==1) 						// echo - 1 -> start timer
	{
		if(rising_edge==0)
		{
			rising_edge=1;
			TCNT0 = 0;
			czas_pomiaru = 0;
		}
		else 							// echo - 0, przelicznik na cm
		{
			rising_edge = 0;
			distance_cm = (czas_pomiaru*256 + TCNT0)/58;
			praca = 0;
		}
	}
}

// timer - on
void wlacz_timer0()
{
	TCCR0 |= (1 << CS00);
	TCNT0 = 0;
	TIMSK |= (1 << TOIE0);
}

// trigger aktywny w pinie D0
void Wyslij_sygnal()
{
	if(praca ==0 )						// czy timer skoñczy³ pracê?
	{
		_delay_ms(50);					//	Restart HC-SR04
		PORTD &=~ (1 << PIND2);
		_delay_us(1);
		PORTD |= (1 << PIND2);			// impuls 10us
		_delay_us(10);
		PORTD &=~ (1 << PIND2);
		praca = 1;
		error = 0;
	}
}

void wlacz_przerwania()					// przerwania na INT1 (echo pin)
{
	MCUCR |= (1 << ISC10);
	GICR |= (1 << INT1);
}



void initPWM(unsigned char value)		//PWM na TIMERZE
{
	TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (1<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
	TCNT1H=0x02;			
	TCNT1L=0x54;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1A = value;			
	OCR1BH=0x00;
	OCR1BL=0x00;
}



void wlacz_uart()							// UART w funkcji Transmittera (CodeVisionAVR) - w³aœciwie chcemy tylko zobaczyæ wart odleg³oœci, nic mu nie wysylamy
{
	UCSRA=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<U2X) | (0<<MPCM);			
	UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);	// TXEN -> 1
	UCSRC=(1<<URSEL) | (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);	// UCSZ1, UCSZ0 -> 1
	UBRRH=0x00;
	UBRRL=0x33;
}

/*void USART_Transmit( char distance_char )				
{
	while ( !( UCSRA & (1<<UDRE)) ) {};
	
		tu wpisac I lub II wersje z maina jako funkcje zdeklarowana wyzej ??
}
*/

int main(void)
{
	wlacz_przerwania();
	wlacz_timer0();
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 
	DDRD &=~ (1 << PIND3);								 // INT1
	DDRD |= (1 << PIND2);								// TRIGGER
	sei();												// globalne przerwania aktywne
	
	
	UCSRB |= (1 << TXEN);						
	DDRD |= LED_PIN;									// wyjœcie PWM na led: OC1A
	while(1) {
		_delay_ms(1);
		unsigned char value = 1;
		
		value = (1-(distance_cm)) * 255;				// u³amek z odleg³oœci max mno¿ymy razy max wype³nienie
		initPWM(value);									// wlaczamy PWM dla leda

		wlacz_uart();									// funkcja transmittera	TxD = 1			
		
		
		/* transmisja	USART_Transmit();
		*/
		
														// test w mainie				
		//int odleglosc=distance_cm;						// teoretycznie jest volatille, wiec moze zwykly int sie ladnie skonwertuje
		
		/*I wersja		char *distance_char;
						itoa(odleglosc,distance_char,10);
						//uart_puts("Odleglosc od przeszkody wynosi:\n");
						uart_puts(distance_char);
		*/
		


		if ((distance_cm > 0) && (distance_cm < 300) )
		{
				char distance_char[100];				// a moze *distance_char;
						sprintf(distance_char, "%d", distance_cm);
						
						_delay_ms(500);
						uart_puts("Odleglosc od przeszkody wynosi:");
						uart_puts(distance_char);
						uart_puts(" mm\n");
		} 
		else
		{	
		_delay_ms(500);
		uart_puts("Nie wykryto przeszkody!\n");
		};

		Wyslij_sygnal();								// wznownienie triggera
			}
}
