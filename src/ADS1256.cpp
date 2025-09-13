//ADS1256 cpp file
/*
 Name:		ADS1256.cpp
 Created:	2022/07/14
 Author:	Curious Scientist
 Editor:	Notepad++
 Comment: Visit https://curiousscientist.tech/blog/ADS1256-custom-library
 Special thanks to Abraão Queiroz for spending time on the code and suggesting corrections for ESP32 microcontrollers!
*/

#include "Arduino.h"
#include "ADS1256.h"
#include "SPI.h"

//----
volatile bool DRDY_value = false; //It is chosen to be false (HIGH) because that's the default state when a conversion is completed

void DRDY_ISR() 
{
	DRDY_value = true; //Change the DRDY_value to true (LOW). (The ISR was fired by a FALLING signal)
}

void waitForDRDY() //A function that waits for the DRDY status to change via the ISR()
{		
	while (DRDY_value == false) {}
	noInterrupts();
	DRDY_value = false; //Reset the DRDY_value manually to false while disabling the interrupts
	interrupts();	
}

//----

//Constructor
ADS1256::ADS1256(const byte DRDY_pin, const int SPI_speed, const byte SYNC_pin, const byte CS_pin, float VREF)
{	
	_DRDY_pin = DRDY_pin; 
	pinMode(_DRDY_pin, INPUT);	
	
	if(SYNC_pin != 0)
	{
	_SYNC_pin = SYNC_pin;
	pinMode(_SYNC_pin, OUTPUT);
	}
	
	_CS_pin = CS_pin;
	pinMode(_CS_pin, OUTPUT);
	
	_VREF = VREF;
}

//Initialization
void ADS1256::InitializeADC()
{
  //Chip select LOW  
  digitalWriteFast(_CS_pin, LOW);
  
  //Sync pin is also treated if it is defined
  if(_SYNC_pin != 0)
  {
  digitalWriteFast(_SYNC_pin, HIGH); //RESET is set to high  
  }
	
  SPI.begin();	    
	
  attachInterrupt(digitalPinToInterrupt(_DRDY_pin), DRDY_ISR, FALLING); //Make the interrupt fire on FALLING signal	
  
  //Applying arbitrary default values to speed up the starting procedure if the user just want to get quick readouts
  delay(200);
  writeRegister(STATUS_REG, 0b00110110); //BUFEN and ACAL enabled, Order is MSB, rest is read only
  delay(200);
  writeRegister(MUX_REG, 0b00000001); //MUX AIN0+AIN1
  delay(200);
  writeRegister(ADCON_REG, 0b0000000); //ADCON - CLK: OFF, SDCS: OFF, PGA = 0 (+/- 5 V)
  delay(200);
  writeRegister(DRATE_REG, 0b0000010); //100SPS 
  delay(200);
  sendDirectCommand(0b11110000); //Offset and self-gain calibration
  delay(200);
  
  _isAcquisitionRunning = false; //MCU will be waiting to start a continuous acquisition
}

void ADS1256::stopConversion() //Sending SDATAC to stop the continuous conversion
{	
	waitForDRDY(); //SDATAC should be called after DRDY goes LOW (p35. Figure 33)
	SPI.transfer(B00001111); //Send SDATAC to the ADC	
	digitalWriteFast(_CS_pin, HIGH); //We finished the command sequence, so we switch it back to HIGH
	SPI.endTransaction();
	
	_isAcquisitionRunning = false; //Reset to false, so the MCU will be able to start a new conversion
}

void ADS1256::setDRATE(uint8_t drate) //Setting DRATE (sampling frequency)
{		
	writeRegister(DRATE_REG, drate);
	_DRATE = drate;
	delay(200);
}

void ADS1256::setMUX(uint8_t mux) //Setting MUX (input channel)
{	
	writeRegister(MUX_REG, mux);
	_MUX = mux;
	delay(200);
}

void ADS1256::setPGA(uint8_t pga) //Setting PGA (input voltage range)
{	
	_PGA = pga;
	_ADCON = _ADCON | 0b00000111; //Change bit 2-0 to 1, leave rest unchanged 
	_ADCON = _ADCON & _PGA; //Combine the new _ADCON with the _PGA (changes bit 0-2)
	writeRegister(ADCON_REG, _ADCON);	
	delay(200);
}

void ADS1256::setCLKOUT(uint8_t clkout) //Setting CLKOUT 
{
	//Values: 0, 1, 2, 3
	
	if(clkout == 0)
	{
		//00
		bitWrite(_ADCON, 6, 0);
		bitWrite(_ADCON, 5, 0);		
	}
	else if(clkout == 1)
	{
		//01 (default)	
		bitWrite(_ADCON, 6, 0);
		bitWrite(_ADCON, 5, 1);			
	}
	else if(clkout == 2)
	{
		//10
		bitWrite(_ADCON, 6, 1);
		bitWrite(_ADCON, 5, 0);		
	}
	else if(clkout == 3)
	{
		//11	
		bitWrite(_ADCON, 6, 1);
		bitWrite(_ADCON, 5, 1);				
	}
	else{}
	
	writeRegister(ADCON_REG, _ADCON);
	delay(100);
}

void ADS1256::setByteOrder(uint8_t byteOrder) //Setting byte order (MSB/LSB)
{
	if(byteOrder == 0)
	{
		//Byte order is MSB (default)
		bitWrite(_STATUS, 3, 0);
		//Set value of _STATUS at the third bit to 0
	}
	else if(byteOrder == 1)
	{
		//Byte order is LSB
		bitWrite(_STATUS, 3, 1);
		//Set value of _STATUS at the third bit to 1
	}
	else{}
	
	writeRegister(STATUS_REG, _STATUS);
	delay(100);
}

void ADS1256::setAutoCal(uint8_t acal) //Setting ACAL (Automatic SYSCAL)
{
	if(acal == 0)
	{
		//Auto-calibration is disabled (default)
		bitWrite(_STATUS, 2, 0);
		//_STATUS |= B00000000;
	}
	else if(acal == 1)
	{
		//Auto-calibration is enabled
		bitWrite(_STATUS, 2, 1);
		//_STATUS |= B00000100;
	}
	else{}
	
	writeRegister(STATUS_REG, _STATUS);
	delay(100);
}

void ADS1256::setBuffer(uint8_t bufen) //Setting input buffer (Input impedance)
{
	if(bufen == 0)
	{
		//Analog input buffer is disabled (default)
		//_STATUS |= B00000000;
		bitWrite(_STATUS, 1, 0);
	}
	else if(bufen == 1)
	{
		//Analog input buffer is enabled (recommended)
		//_STATUS |= B00000010;
		bitWrite(_STATUS, 1, 1);
	}
	else{}
	
	writeRegister(STATUS_REG, _STATUS);
	delay(100);
}

void ADS1256::sendDirectCommand(uint8_t directCommand)
{
  //Direct commands can be found in the datasheet Page 34, Table 24.
  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE1));

  digitalWriteFast(_CS_pin, LOW); //REF: P34: "CS must stay low during the entire command sequence"
  delayMicroseconds(5);
  SPI.transfer(directCommand); //Send Command
  delayMicroseconds(5);
  digitalWriteFast(_CS_pin, HIGH); //REF: P34: "CS must stay low during the entire command sequence"

  SPI.endTransaction();
}

float ADS1256::convertToVoltage(int32_t rawData) //Converting the 24-bit data into a micro-voltage value
{	
  // if (rawData >> 23 == 1) //if the 24th digit (sign) is 1, the number is negative
  // {
  //   rawData = rawData - 16777216;  //conversion for the negative sign
  //   //"mirroring" around zero
  // }  
  
  float voltage = ((2 * _VREF) / 8388608) * rawData / (pow(2, _PGA))*1000000; //8388608 = 2^{23} - 1  
  //REF: p23, Table 16.
	
  return(voltage);
}

void ADS1256::writeRegister(uint8_t registerAddress, uint8_t registerValueToWrite)
{		
  waitForDRDY();
  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE1));
  //SPI_MODE1 = output edge: rising, data capture: falling; clock polarity: 0, clock phase: 1.

  digitalWriteFast(_CS_pin, LOW); //CS must stay LOW during the entire sequence [Ref: P34, T24]
  	
  delayMicroseconds(6); //see t6 in the datasheet

  SPI.transfer((0x50 & 0b11110000) | registerAddress); // 0x50 = 01010000 = WREG

  SPI.transfer(0x00); //2nd (empty) command byte
  	
  SPI.transfer(registerValueToWrite); //pass the value to the register

  delayMicroseconds(6);
  
  digitalWriteFast(_CS_pin, HIGH);
  SPI.endTransaction();
  delay(100);
  
}

long ADS1256::readRegister(uint8_t registerAddress) //Reading a register
{
   waitForDRDY();
	
  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE1));
  //SPI_MODE1 = output edge: rising, data capture: falling; clock polarity: 0, clock phase: 1.

  digitalWriteFast(_CS_pin, LOW); //CS must stay LOW during the entire sequence [Ref: P34, T24]

  SPI.transfer(0x10 | registerAddress); //0x10 = 0001000 = RREG - OR together the two numbers (command + address)

  SPI.transfer(0x00); //2nd (empty) command byte

  delayMicroseconds(6); //see t6 in the datasheet

  _registerValuetoRead = SPI.transfer(0xFF); //read out the register value

  digitalWriteFast(_CS_pin, HIGH);
  SPI.endTransaction();
  delay(100);
  return _registerValuetoRead;
}


long ADS1256::readSingle() //Reading a single value ONCE using the RDATA command 
{
	SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE1));
	digitalWriteFast(_CS_pin, LOW); //REF: P34: "CS must stay low during the entire command sequence"  
	waitForDRDY();
	SPI.transfer(B00000001); //Issue RDATA (0000 0001) command
	delayMicroseconds(7); //Wait t6 time (~6.51 us) REF: P34, FIG:30.

	_outputBuffer[0] = SPI.transfer(0); // MSB
	_outputBuffer[1] = SPI.transfer(0); // Mid-byte
	_outputBuffer[2] = SPI.transfer(0); // LSB		

	//Shifting and combining the above three items into a single, 24-bit number
	_outputValue = ((long)_outputBuffer[0]<<16) | ((long)_outputBuffer[1]<<8) | (_outputBuffer[2]);
	
	digitalWriteFast(_CS_pin, HIGH); //We finished the command sequence, so we set CS to HIGH
	SPI.endTransaction();
  
	return(_outputValue);
}

long ADS1256::readSingleContinuous() //Reads the recently selected input channel using RDATAC
{		
	if(_isAcquisitionRunning == false)
	{
	  _isAcquisitionRunning = true;
	  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE1));
	  digitalWriteFast(_CS_pin, LOW); //REF: P34: "CS must stay low during the entire command sequence"	  
	  while (digitalReadFast(_DRDY_pin)) {}
	  SPI.transfer(B00000011);  //Issue RDATAC (0000 0011) 
	  delayMicroseconds(7); //Wait t6 time (~6.51 us) REF: P34, FIG:30.
	  
	}
	else
	{
		waitForDRDY();
	}	
	
	noInterrupts();
	_outputBuffer[0] = SPI.transfer(0); // MSB 
	_outputBuffer[1] = SPI.transfer(0); // Mid-byte
	_outputBuffer[2] = SPI.transfer(0); // LSB
	interrupts();
	_outputValue = ((long)_outputBuffer[0]<<16) | ((long)_outputBuffer[1]<<8) | (_outputBuffer[2]);
	
	return _outputValue;
}

long ADS1256::cycleSingle()
{
	if(_isAcquisitionRunning == false)
	{
	  _isAcquisitionRunning = true;
	  _cycle = 0;
	  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE1));
	  digitalWriteFast(_CS_pin, LOW); //CS must stay LOW during the entire sequence [Ref: P34, T24]
	  SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
      SPI.transfer(0x00);
      SPI.transfer(SING_0); //AIN0+AINCOM
	  digitalWriteFast(_CS_pin, HIGH);			
	  delay(50);
	  digitalWriteFast(_CS_pin, LOW); //CS must stay LOW during the entire sequence [Ref: P34, T24]
	}
	else
	{}
  
	if(_cycle < 8)
	{      
	  _outputValue = 0;
	  waitForDRDY();
      //Step 1. - Updating MUX
      switch (_cycle)
      {
        noInterrupts();
        //Channels are written manually
        case 0: //Channel 2
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(SING_1);  //AIN1+AINCOM
          break;

        case 1: //Channel 3
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(SING_2);  //AIN2+AINCOM
          break;

        case 2: //Channel 4
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(SING_3);  //AIN3+AINCOM
          break;

        case 3: //Channel 5
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(SING_4);  //AIN4+AINCOM
          break;

        case 4: //Channel 6
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(SING_5);  //AIN5+AINCOM
          break;

        case 5: //Channel 7
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(SING_6);  //AIN6+AINCOM
          break;

        case 6: //Channel 8
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(SING_7);  //AIN7+AINCOM
          break;

        case 7: //Channel 1
          SPI.transfer(0x50 | 1); // 0x50 = WREG //1 = MUX
          SPI.transfer(0x00);
          SPI.transfer(SING_0); //AIN0+AINCOM
          break;
      }
      //Step 2.
      SPI.transfer(B11111100); //SYNC
      delayMicroseconds(4); //t11 delay 24*tau = 3.125 us //delay should be larger, so we delay by 4 us
      SPI.transfer(B11111111); //WAKEUP

      //Step 3.
      //Issue RDATA (0000 0001) command
      SPI.transfer(B00000001);
      delayMicroseconds(7); //Wait t6 time (~6.51 us) REF: P34, FIG:30.

	  _outputBuffer[0] = SPI.transfer(0x0F); // MSB 
	  _outputBuffer[1] = SPI.transfer(0x0F); // Mid-byte
	  _outputBuffer[2] = SPI.transfer(0x0F); // LSB
		
	  _outputValue = ((long)_outputBuffer[0]<<16) | ((long)_outputBuffer[1]<<8) | (_outputBuffer[2]);
		
	  _cycle++; //Increase cycle - This will move to the next MUX input channel
	  if(_cycle == 8)
	  {
		  _cycle = 0; //Reset to 0 - Restart conversion from the 1st input channel
	  }
    }
  	interrupts();
	return _outputValue;
}