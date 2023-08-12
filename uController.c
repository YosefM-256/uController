#include <ADuC841.H>

#define disableADCinterrupt {ADCI = 0; EADC = 0;}
#define enableADCinterrupt {ADCI = 0; EADC = 1;}
#define powerOnADC {ADCCON1 |= 0x80;}
#define enableInterrupts {EA=1;}
#define enableUARTinterrupt {ES = 1;}
#define enableTimer2Interrupt {ET2 = 1;}
#define disableTimer2Interrupt {ET2 = 0;}
#define enableInternalXRAM {CFG841 |= 0x01;}

#define DATAseg 0
#define XDATAseg 1
#define CODEseg 2

#define BAUDRATE 115200
#define THVALUE (uint8) (256 - 11.0592E6/(16*BAUDRATE))
#define tickDelay 0.5E-3
#define RCAP2Hval (int)(65536 - tickDelay*11.0592E6)/256
#define RCAP2Lval (int)(65536 - tickDelay*11.0592E6)%256	
#define INPUTBUFFERSIZE 200
#define OUTPUTBUFFERSIZE 800

#define ENDDATA 0x80
#define ENDCOMMS 0x81

typedef unsigned char uint8;
typedef unsigned int uint16;
typedef int int16;


volatile uint8 xdata 	outputBuffer[OUTPUTBUFFERSIZE];
volatile int16 		outputBufferInd = 0;
volatile uint8 xdata 	inputBuffer[INPUTBUFFERSIZE];
volatile int16 		inputBufferInd = 0;
volatile int16	 	executionInd = 0;
volatile uint16 	sentInd = 0;
volatile uint8 		sentIndBit = 0;
volatile int16 		timer2counter;

volatile uint8 bdata 	bools = 0;

sbit pendingADCinterrupt = 	bools^0;
sbit execute = 			bools^1;
sbit doneSending = 		bools^2;

void setupUART();
void getADC(uint8 ADCnum);
void setupADC(uint8 ADCnum);
void ADCisr();
void timer2isr();
void setDAC(uint8 DACnum, uint16 DACvalue);
void setPin(uint8 port, uint8 pin, uint8 state);
void setupUART();
void UARTisr();
void receiveByte();
void sendByte();
void runCommands();
void sendMsg(uint8 msg);
void com_setDAC(uint8 xdata *inBuffer);
void com_readADC(uint8 xdata *inBuffer);
void com_getAllADCs(uint8 xdata *inBuffer);
void com_setPin(uint8 xdata *inBuffer);
void com_delay(uint8 xdata *inBuffer);
void com_dischargeBuffer(uint8 xdata *inBuffer);
void com_readMemory(uint8 xdata *inBuffer);
void com_resetBuffers(uint8 xdata *inBuffer);


code void (*com_functions[32])(uint8) = {
	0,0,0,0, 0,0,0,0,
	0,0,0,0, 0,0,0,0,
	&com_setDAC,
	&com_readADC,
	&com_dischargeBuffer,
	&com_getAllADCs,
	&com_setPin,
	&com_delay,
	&com_readMemory,
	&com_resetBuffers,
	0,0, 0,0,0,0,
	0,0};
	
int code com_bytesNum[32] = {
	0,0,0,0, 0,0,0,0,
	0,0,0,0, 0,0,0,0,
	3,2,1,1, 2,2,3,1,
	0,0,0,0, 0,0,0,0};

int main(){
	enableInterrupts
	enableInternalXRAM
	setupUART();
	
	while(1){
		if (execute)
			runCommands();
	}
	return 0;
}
	

void runCommands(){
	while (executionInd < inputBufferInd){
		uint8 opCodeByte = inputBuffer[executionInd];
		opCodeByte = (opCodeByte >> 2) & 0x1F;
		(*com_functions[opCodeByte])(inputBuffer+executionInd);
		
		executionInd += com_bytesNum[opCodeByte];
	}
	sendMsg(ENDCOMMS);
	executionInd = 0;
	inputBufferInd = 0;
	execute = 0;
	REN = 1;
}


void sendMsg(uint8 msg) {
	doneSending = 0;
	SBUF = msg;
	while (!doneSending);
}


void UARTisr() interrupt 4 {
	if (RI){
		RI = 0;
		receiveByte();
	}
	if (TI) {
		TI = 0;
		sendByte();
	}
}


void ADCisr() interrupt 6 {
	outputBuffer[outputBufferInd++] = ADCDATAH;
	outputBuffer[outputBufferInd++] = ADCDATAL;
	disableADCinterrupt
	pendingADCinterrupt = 0;
}


void timer2isr() interrupt 5 {
	TF2 = 0;
	timer2counter--;
}


void setupADC(uint8 ADCnum){
	enableInterrupts
	enableADCinterrupt
	ADCCON1 = 0x8C;
	
	ADCnum &= 0x07;
	ADCCON2 |= ADCnum;
	
	ADCnum |= 0xF8;
	ADCCON2 &= ADCnum;
	
	SCONV = 1;
}


void getADC(uint8 ADCnum){
	pendingADCinterrupt = 1;
	setupADC(ADCnum);
	while(pendingADCinterrupt);
}	

/* This function disables any UART interrupt_ that was pending.
   It uses the UART in mode 1 with timer 1 to set the baud rate. */
void setupUART(){
	SCON = 0x50; 
	TMOD = 0x20;
	
	TH1   = THVALUE;	
        TR1   = 1;  
	PCON  |= 0x80;
	
	RI = 0;
	TI = 0;
	enableInterrupts
	enableUARTinterrupt
}


void setDAC(uint8 DACnum, uint16 DACvalue){
	powerOnADC // this powers on the ADC
	
	DACvalue &= 0x0FFF;
	DACnum &= 0x1;
	DACCON = 0x7B;
	
	if (DACnum){
		DAC1H = (uint8)(DACvalue >> 8);
		DAC1L = (uint8)(DACvalue & 0x00FF);
	}
	else{
		DAC0H = (uint8)(DACvalue >> 8);
		DAC0L = (uint8)(DACvalue & 0x00FF);
	}
	
	DACCON |= 0x04; // set the sync in the DACCON, making the changes to the DACs effective.
}
	
	
void setPin(uint8 port, uint8 pin, uint8 state){
	//SBUF = 'e';
	if (port == 3){
		if (state == 1)
			P3 |= (1 << pin);
		if (state == 0)
			P3 &= ~(1 << pin);
	}
	if (port == 2){
		if (state == 1)
			P2 |= (1 << pin);
		if (state == 0)
			P2 &= ~(1 << pin);
	}
	if (port == 0){
		if (state == 1)
			P0 |= (1 << pin);
		if (state == 0)
			P0 &= ~(1 << pin);
	}	
}


void receiveByte(){
	uint8 newByte;
	//SBUF = 'b';
	newByte = SBUF;
	// add here code that deals with buffer overfow
	inputBuffer[inputBufferInd] = newByte;
	inputBufferInd++;
	if (newByte & 0x80){	// end of command - now starts the execution
		//SBUF = 'c';
		REN = 0;
		execute = 1;
	}
}


void sendByte(){
	if (sentInd >= outputBufferInd){
		//as for now, there's only ENDCOMMS message
		//SBUF = ENDDATA;
		doneSending = 1;
		return;
	}
	else {
		uint8 leftHalf;
		uint8 rightHalf;
		uint8 toSend;
		if (sentIndBit == 0){
			leftHalf = outputBuffer[sentInd] >> 1;
		}
		else {
			leftHalf = ( outputBuffer[sentInd] << (sentIndBit-1) );
		}
		
		if (sentInd + 1 >= outputBufferInd) {
			rightHalf = 0;
		}
		else {
			rightHalf = ( outputBuffer[sentInd + 1] >> (8 - sentIndBit + 1) );
		}
		
		toSend = (leftHalf | rightHalf) & 0x7F;
		
		SBUF = toSend;
		sentInd = sentInd + (sentIndBit+7)/8;
		sentIndBit = (sentIndBit+7)%8;
	}
}


void com_setDAC(uint8 xdata *inBuffer){
	uint8 DACnum = inBuffer[0] & 0x01;
	uint8 DACvalueBytes[2];
	uint16 DACvalue = 0;
	DACvalueBytes[0] = (inBuffer[1] & 0x1F) >> 1;
	DACvalueBytes[1] = ( (inBuffer[1] & 0x01) << 7 ) | (inBuffer[2] & 0x7F);
	DACvalue = *( (uint16*)DACvalueBytes );
	setDAC(DACnum, DACvalue);
}


void com_readADC(uint8 xdata *inBuffer){
	uint8 times = (inBuffer[1] & 0x78) >> 3;
	uint8 ADCnum = (inBuffer[1] & 0x07);
	int i;
	for (i = 0; i < times; i++){
		getADC(ADCnum);
	}
}


void com_getAllADCs(uint8 xdata *inBuffer){
	uint8 times = (1 << (inBuffer[0] & 0x03));
	
	uint8 time, adcNum;
	for(time = 0; time < times; time++){
		for (adcNum = 0; adcNum < 8; adcNum++){
			getADC(adcNum);
		}
	}
}


void com_setPin(uint8 xdata *inBuffer){
	uint8 port = inBuffer[0] & 0x03;
	uint8 pinNumber = inBuffer[1] & 0x07;
	uint8 state = (inBuffer[1] & 0x08) >> 3;
	setPin(port, pinNumber, state);
	//SBUF = 'd';
}


void com_delay(uint8 xdata *inBuffer){
	uint8 delay = inBuffer[1] & 0x7F;
	
	enableInterrupts
	enableTimer2Interrupt
		
	timer2counter = delay;
	RCAP2H = RCAP2Hval;
	RCAP2L = RCAP2Lval;
	T2CON = 0x04;
	
	while(timer2counter >= 0);
	
	disableTimer2Interrupt
}	
	

void com_dischargeBuffer(uint8 xdata *inBuffer){
	doneSending = 0;
	TI = 1;
	while (!doneSending);
	sentInd = 0;
	sentIndBit = 0;
	outputBufferInd = 0;
}
	
	
void com_readMemory(uint8 xdata *inBuffer){
	uint8 segment = inBuffer[0] & 0x03;
	uint16 address = ( ( (*(uint16*)(inBuffer+1)) & 0x7f00 ) >> 1 ) |
			( (*(uint16*)(inBuffer+1)) & 0x007f );
	if (segment == DATAseg)
		outputBuffer[outputBufferInd++] = *(uint8 data *)(address);
	if (segment == XDATAseg)
		outputBuffer[outputBufferInd++] = *(uint8 xdata *)(address);
	if (segment == CODEseg)
		outputBuffer[outputBufferInd++] = *(uint8 code *)(address);
}
	
	
void com_resetBuffers(uint8 xdata *inBuffer){
	inputBufferInd = 0;
	outputBufferInd = 0;
}
	





















