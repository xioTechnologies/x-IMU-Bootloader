//=====================================================================================================
// main.c
// S.O.H. Madgwick
// <date>
//=====================================================================================================
//
// Modified code from Microchip application notes AN851.
//
// Bootloader time out is specfied in firmware .gld file as line:
// SHORT(0x01);	/* Bootloader timeout in sec */
//
//====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include <p33FJ128GP804.h>

//---------------------------------------------------------------------------------------------------
// Configuration Bits

_FOSCSEL(FNOSC_FRCPLL)		// Fast RC oscillator
_FOSC(OSCIOFNC_ON)			// OSCO pin has digital I/O function
_FWDT(FWDTEN_OFF)			// Watchdog Timer: Disabled

//---------------------------------------------------------------------------------------------------
// Definitions

#define bluetoothEnablePin _LATA9

#define FCY   			    29480000                
#define BRGVAL              15

#define COMMAND_NACK        0x00
#define COMMAND_ACK         0x01
#define COMMAND_READ_PM     0x02
#define COMMAND_WRITE_PM    0x03
#define COMMAND_WRITE_CM    0x07
#define COMMAND_RESET       0x08
#define COMMAND_READ_ID     0x09

#define PM_ROW_SIZE         64 * 8
#define CM_ROW_SIZE         8
#define CONFIG_WORD_SIZE    1

#define PM_ROW_ERASE 		0x4042
#define PM_ROW_WRITE 		0x4001
#define CONFIG_WORD_WRITE	0X4000

typedef short               Word16;
typedef unsigned short      UWord16;
typedef long                Word32;
typedef unsigned long       UWord32;

typedef union tuReg32 {
	UWord32 Val32;
	struct 	{
		UWord16 LW;
		UWord16 HW;
	} Word;
	char Val[4];
} uReg32;

//---------------------------------------------------------------------------------------------------
// Variable declaration and definitions

char Buffer[PM_ROW_SIZE*3 + 1];

//---------------------------------------------------------------------------------------------------
// Function declarations

int main(void);
void initMain(void);
void initRapidFlashLED(void);

extern UWord32 ReadLatch(UWord16, UWord16);
void PutChar(char);
void GetChar(char *);
void WriteBuffer(char *, int);
void ReadPM(char *, uReg32);
void WritePM(char *, uReg32);

//====================================================================================================
// Functions
//====================================================================================================

//---------------------------------------------------------------------------------------------------
// Main routine

int main(void) {
    uReg32 SourceAddr;
	uReg32 Delay;
	
	initMain();
	initRapidFlashLED();
	bluetoothEnablePin = 1;

	RCONbits.SWDTEN=0;                                              // Disable Watch Dog Timer
	while(OSCCONbits.LOCK!=1) {};                                   // Wait for PLL to lock
	SourceAddr.Val32 = 0xc00;
	Delay.Val32 = ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);
	if(Delay.Val[0] == 0) ResetDevice();
	T4CONbits.T32 = 1;		                                        // to increment every instruction cycle
	IFS1bits.T5IF = 0;	                                         	// Clear the Timer3 Interrupt Flag
	IEC1bits.T5IE = 0;		                                        // Disable Timer3 Interrupt Service Routine
	if((Delay.Val32 & 0x000000FF) != 0xFF) {
		Delay.Val32 = ((UWord32)(FCY)) * ((UWord32)(Delay.Val[0])); // Convert seconds into timer count value
		PR5 = Delay.Word.HW;
		PR4 = Delay.Word.LW;
		T4CONbits.TON=1;                                            // Enable Timer
	}
	U1BRG = BRGVAL;                                                 // BAUD Rate Setting of UART
	U1MODE = 0x8000;                                                // Reset UART to 8-n-1, alt pins, and enable
	U1STA  = 0x0400;                                                // Reset status register and enable TX

	while(1) {
		char Command;
		GetChar(&Command);
		switch(Command) {
			case COMMAND_READ_PM:				                    // tested
			{
				uReg32 SourceAddr;
				GetChar(&(SourceAddr.Val[0]));
				GetChar(&(SourceAddr.Val[1]));
				GetChar(&(SourceAddr.Val[2]));
				SourceAddr.Val[3]=0;
				ReadPM(Buffer, SourceAddr);
				WriteBuffer(Buffer, PM_ROW_SIZE*3);
				break;
			}
			case COMMAND_WRITE_PM:				                    // tested
			{
			    uReg32 SourceAddr;
				int Size;
				GetChar(&(SourceAddr.Val[0]));
				GetChar(&(SourceAddr.Val[1]));
				GetChar(&(SourceAddr.Val[2]));
				SourceAddr.Val[3]=0;
				for(Size = 0; Size < PM_ROW_SIZE*3; Size++) {
				    GetChar(&(Buffer[Size]));
				}
				Erase(SourceAddr.Word.HW,SourceAddr.Word.LW,PM_ROW_ERASE);
				WritePM(Buffer, SourceAddr);	                    // program page
				PutChar(COMMAND_ACK);			                    // Send Acknowledgement
 				break;
			}
			case COMMAND_READ_ID:
			{
				uReg32 SourceAddr;
				uReg32 Temp;
				SourceAddr.Val32 = 0xFF0000;
				Temp.Val32 = ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);
				WriteBuffer(&(Temp.Val[0]), 4);
				SourceAddr.Val32 = 0xFF0002;
				Temp.Val32 = ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);
				WriteBuffer(&(Temp.Val[0]), 4);
				break;
			}
			case COMMAND_WRITE_CM:	
			{
				int Size;
				for(Size = 0; Size < CM_ROW_SIZE*3;) {
					GetChar(&(Buffer[Size++]));
					GetChar(&(Buffer[Size++]));
					GetChar(&(Buffer[Size++]));
					PutChar(COMMAND_ACK);		                    // Send Acknowledgement
				}
				
				break;
			}
		    case COMMAND_RESET:
			{
				uReg32 SourceAddr;
				int    Size;
				uReg32 Temp;
				for(Size = 0, SourceAddr.Val32 = 0xF80000; Size < CM_ROW_SIZE*3; Size +=3, SourceAddr.Val32 += 2) {
					if(Buffer[Size] == 0) {
						Temp.Val[0]=Buffer[Size+1];
						Temp.Val[1]=Buffer[Size+2];
						WriteLatch( SourceAddr.Word.HW, SourceAddr.Word.LW, Temp.Word.HW, Temp.Word.LW);
						WriteMem(CONFIG_WORD_WRITE);
					}
				}
				ResetDevice();
				break;
			}
			case COMMAND_NACK:
			{
				ResetDevice();
				break;
			}
			default:
				PutChar(COMMAND_NACK);
				break;
		}
	}
}

//---------------------------------------------------------------------------------------------------
// Initialise I/O, oscillator and other core registers
// Copied from firmware source (some lines commented out)

void initMain(void) {   

	/* Init I/Os */
	PORTA = 0x0000; 							// clear all PORT
	PORTB = 0x0000;
	PORTC = 0x0000;
	LATA = 0x0000;								// clear all LAT
	LATB = 0x0000;
	LATC = 0x0000;
	TRISA = 0x040D;								// setup all TRIS
	TRISB = 0xFA8F;
	TRISC = 0x023C;
	AD1PCFGL = 0xFFFE;							// configure all I/O pins to be digital except AN1 (IMPORTANT: value lost if ADC peripheral disabled)

	/* Map peripherals */
	//unmapAllPeripherals();                      // un-map all peripherals as bootloader may use different settings
	asm volatile("BCLR OSCCON, #6");			// unlock the control registers
	_RP17R = 0b00011;							// RP17 (RC1) mapped to U1TX (FTDI RX)
	_U1RXR = 19;								// RP19 (RC3) mapped to U1RX (FTDI TX)
	_U1CTSR = 18;								// RP18 (RC2) mapped to U2CTS (FTDI RTS)
	_RP16R = 0b00100;							// RP16 (RC0) mapped to U2RTS (FTDI CTS)
	_RP5R = 0b00101;							// RP5 (RB5) mapped to U2TX (Bluetooth RX)
	_U2RXR = 21;								// RP21 (RC5) mapped to U2RX (Bluetooth TX)
	_U2CTSR = 20;								// RP20 (RC4) mapped to U2CTS (Bluetooth RTS)
	_RP6R = 0b00110;							// RP6 (RB6) mapped to U2RTS (Bluetooth CTS)
	_INT1R = 11;								// RP11 (RB11) mapped to INT1 (sleep/wake button)
	_RP24R = 0b01000;							// RP24 (RC8) mapped to SCK1 (SD card CLK)
	_SDI1R = 25;								// RP25 (RC9) mapped to SDI2 (uSD card DOUT)
	_RP23R = 0b00111;							// RP23 (RC7) mapped to SDO2 (uSD card DIN)
	asm volatile("BSET OSCCON, #6");			// lock the control registers

	/* Misc. inits */
	__builtin_write_OSCCONL(OSCCON | 0x0002);	// enable secondary 32.768 kHz oscillator
	_NSTDIS = 0;								// interrupt nesting enabled

	///* Disable power to all peripherals */
	//PMD1 = 0xFFFE;								// do not disable ADC
	//PMD2 = 0xFFFF;
	//PMD3 = 0xFDFF;								// do not disable RTCC

	/* Setup PLL for at 29.48 MIPS */
	_PLLPRE = 0;
	_PLLDIV = 30;
	_PLLPOST = 0;
	while(!OSCCONbits.LOCK);					// wait for PLL to lock
}

//---------------------------------------------------------------------------------------------------
// Rapid flash LED

void initRapidFlashLED(void) {

    /* Map OC1 peripheral */
    asm volatile("BCLR OSCCON, #6");    // unlock the control registers
		_RP10R = 0b10010;			    // RP10 (RB10) mapped to OC1 (LED)
	asm volatile("BSET OSCCON, #6");    // lock the control registers
	
	/* Setup TMR2 */
    T2CON = 0x0000;			            // clear register
    T2CONbits.TCKPS = 0b10;             // 1:64 prescaler
    T2CONbits.TON = 1;                  // start TMR2
    
    /* Setup OC1 as PWM */
    OC1CON = 0x0000;                    // clear register
    OC1CONbits.OCTSEL = 0;              // TMR2 is the clock source
    OC1CONbits.OCM = 0b110;             // PWM mode, Fault pin disabled
    OC1RS = 0x8000;                     // 50% duty cycle
}    

//---------------------------------------------------------------------------------------------------
// Subroutines

void GetChar(char * ptrChar) {
	while(1) {
		if(IFS1bits.T5IF == 1) {        // if timer expired, signal to application to jump to user code
			* ptrChar = COMMAND_NACK;
			break;
		}
		if(U1STAbits.FERR == 1)	{       // check for receive errors
			continue;
		}
		if(U1STAbits.OERR == 1)	{       // must clear the overrun error to keep uart receiving
			U1STAbits.OERR = 0;
			continue;
		}
		if(U1STAbits.URXDA == 1) {      // get the data
			T4CONbits.TON=0;            // Disable timer countdown
			* ptrChar = U1RXREG;
			break;
		}
	}
}

void ReadPM(char * ptrData, uReg32 SourceAddr) {
	int Size;
	uReg32 Temp;
	for(Size = 0; Size < PM_ROW_SIZE; Size++) {
		Temp.Val32 = ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);
		ptrData[0] = Temp.Val[2];;
		ptrData[1] = Temp.Val[1];;
		ptrData[2] = Temp.Val[0];;
		ptrData = ptrData + 3;
		SourceAddr.Val32 = SourceAddr.Val32 + 2;
	}
}

void WriteBuffer(char * ptrData, int Size) {
	int DataCount;	
	for(DataCount = 0; DataCount < Size; DataCount++) {
		PutChar(ptrData[DataCount]);
	}
}

void PutChar(char Char) {
	while(!U1STAbits.TRMT);
	U1TXREG = Char;
}

void WritePM(char * ptrData, uReg32 SourceAddr) {
	int Size,Size1;
	uReg32 Temp;
	uReg32 TempAddr;
	uReg32 TempData;
	for(Size = 0,Size1=0; Size < PM_ROW_SIZE; Size++) {
		Temp.Val[0]=ptrData[Size1+0];
		Temp.Val[1]=ptrData[Size1+1];
		Temp.Val[2]=ptrData[Size1+2];
		Temp.Val[3]=0;
		Size1+=3;
	   	WriteLatch(SourceAddr.Word.HW, SourceAddr.Word.LW,Temp.Word.HW,Temp.Word.LW);

		/* Device ID errata workaround: Save data at any address that has LSB 0x18 */
		if((SourceAddr.Val32 & 0x0000001F) == 0x18) {
			TempAddr.Val32 = SourceAddr.Val32;
			TempData.Val32 = Temp.Val32;
		}
		if((Size !=0) && (((Size + 1) % 64) == 0)) {
    		
            /* Device ID errata workaround: Reload data at address with LSB of 0x18 */
	        WriteLatch(TempAddr.Word.HW, TempAddr.Word.LW,TempData.Word.HW,TempData.Word.LW);
			WriteMem(PM_ROW_WRITE);
		}
		SourceAddr.Val32 = SourceAddr.Val32 + 2;
	}
}

//====================================================================================================
// END OF CODE
//====================================================================================================
