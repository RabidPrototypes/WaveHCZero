
/*
 William Greiman's modified version of Ladyada's wave shield library
 I have made many changes that may have introduced bugs.  Major changes are:
 
 + optimized DAC macros to allow 44.1 k 16-bit files
 + use of FatReader to read FAT32 and FAT16 files
 + modified readwavhack to be readWaveData
 + use standard SD and SDHC flash cards.
 + skip non-data chunks after fmt chunk
 + allow 18 byte format chunk if no compression
 + play stereo as mono by interleaving channels
 + change method of reading fmt chunk - use union of structs
 
 The following changes were made by Shawn Swift:
 
 + Added the ability to loop sound effects.
 + Added a global volume with 4096 levels and slew-rate limiting to avoid pops when first starting up and when changing the volume level.
 + Added peak level detection.
 + Ported to the Arduino Zero + latest SD/FAT library.

*/

#include <string.h>
#include <WaveHCZero.h>
#include <SPI.h>

// verify program assumptions
#if PLAYBUFFLEN != 256 && PLAYBUFFLEN != 512
#error PLAYBUFFLEN must be 256 or 512
#endif // PLAYBUFFLEN

WaveHC *playing = 0;

uint8_t buffer1[PLAYBUFFLEN];
uint8_t buffer2[PLAYBUFFLEN];
uint8_t *playend;      // end position for current buffer
uint8_t *playpos;      // position of next sample
uint8_t *sdbuff;       // SD fill buffer
uint8_t *sdend;        // end of data in sd buffer

// status of sd
#define SD_READY 1     // buffer is ready to be played
#define SD_FILLING 2   // buffer is being filled from DS
#define SD_END_FILE 3  // reached end of file
uint8_t sdstatus = 0;

// Peak detecton

uint16_t peakMin = 65535;
uint16_t peakMax = 0;

// Volume
uint16_t volume = 4095; // Global volume level.
uint16_t SRL_volume = 0; // Slew-rate limited volume.

uint8_t SPI_busy = 0; // If this flag is set the DAC interrupt will skip the next sample.

// Beware hidden spaces after \'s in defines!
// I guess no ; is needed after the asm because when you call MultiU8X16to24() you will put one after it, and that will just be replaced with the code below.


/* 
This function multiplies two unsigned 16 bit numbers together and places the 32-bit result in longRes.
*/

#define MulU16X16to32(longRes, intIn1, intIn2) \
asm volatile ( \
"clr r16 \n\t" \
"mul %A1, %A2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %B1, %B2 \n\t" \
"movw %C0, r0 \n\t" \
"mul %B2, %A1 \n\t" \
"add %B0, r0 \n\t" \
"adc %C0, r1 \n\t" \
"adc %D0, r16 \n\t" \
"mul %B1, %A2 \n\t" \
"add %B0, r0 \n\t" \
"adc %C0, r1 \n\t" \
"adc %D0, r16 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (longRes) \
: \
"a" (intIn1), \
"a" (intIn2) \
: \
"r16" \
) 


//------------------------------------------------------------------------------
// timer interrupt for DAC
ISR(TIMER3_COMPA_vect) {
  
  //if (SPI_busy) return;
  
  if (!playing) return;

  if (playpos >= playend) { // If we've reached the end of the buffer...

    if (sdstatus == SD_READY) {
    
      // swap double buffers

      playpos = sdbuff;
      playend = sdend;
      sdbuff = sdbuff != buffer1 ? buffer1 : buffer2;
      
      sdstatus = SD_FILLING;

	 TIMSK3 |= _BV(OCIE3B); // Enable buffer fill interrupt?

    }
    else if (sdstatus == SD_END_FILE) {

      playing->stop();	
      return;

    }
    else {

      // count overrun error if not at end of file

      if (playing->remainingBytesInChunk) {
        playing->errors++;
      }

      return;

    }
  }

  // Note that the following statements assume 16-bit samples.  8-bit samples are not supported at this time.

  //if ((OCR1A != 0) && ((TCNT1+24) > OCR1A)) { playpos += 2; return; } // If servo interrupt is active, check to see if we have time to update the DAC without delaying the servo update. (The servos are sensitive to being updated on time.) Note that servo timer1 is /8, so a delay of 1 timer tick is equivalent to 8 clock cycles.
	

	// Process next 16-bit sample:
		
		// 16-bit sample is signed. 
		// Flipping the most significant bit to converts a signed samples to unsigned. 
		// Ie, if dealing with 8 bit numbers, signed 0 becomes unsigned 128, signed -1 becomes unsigned 127, and signed -128 becomes unsigned 0.  
		
		uint8_t dh, dl;	

		dl = playpos[0];
		dh = 0X80 ^ playpos[1]; // Convert signed sample to unsigned.

		playpos += 2;	


#if DVOLUME

	// Adjust sample by WAV volume:
			
		// 12-bit volume (0..4095):

		uint32_t tmp;
		uint16_t sample = (dh << 8) | dl;

		//MulU16X16to32(tmp, playing->volume, sample);
		//sample = tmp>>16; // New 12-bit sample is in upper 16 bits of tmp.  Shift right should be optimized out.	

	// Do peak level detection.
	// We want the peak level to be affected by the local wav volume, but not the global volume.
		if (peakMin > sample) { peakMin = sample; }
		if (peakMax < sample) { peakMax = sample; }

	// Adjust by global volume level.
	// Do so after the peak detector so that the animated display doesn't change as the volume is turned down.
		MulU16X16to32(tmp, SRL_volume, sample);
		sample = tmp>>16; // New 12-bit sample is in upper 16 bits of tmp.  Shift right should be optimized out.
			
     // Take the SS pin low to select the DAC. (Bit 4 on port B is the hardware SPI slave select pin on the Atmega1284p)

     //		PORTB &= ~0b10000; 

     // Send 16 bits to the DAC.
	// First 4 bits are configuration bits. xx00 = (Don't care, Don't care, not muted)
	// Last 12 bits specify the output voltage. (MSB first)

	// The 12-bit sample is stored in a single 16 bit integer.

    		//SPDR = 0b00000000 | (sample >> 8); 
    		//while (!(SPSR & _BV(SPIF)));
		
    		//SPDR = sample; 
    		//while (!(SPSR & _BV(SPIF)));
		
		//mcpDacSend(0);
		mcpDacSend(sample);

	// Adjust slew-rate limited volume level.
	
		// Slew-rate limiting the volume prevents the DAC which starts out at 0V from jumping up to VCC/2 instantly when the first WAV file is played.
		// It also serves to smooth out any noise or discontinuities in the volume level when adjusting it via a potentiometer.

		if (SRL_volume != volume) { if (SRL_volume < volume) { SRL_volume++; } else { SRL_volume--; } } // Adjust volume level by one tick per sample played until we reach the desired volume level. This will takes 1/5th of a second in the worst case.  

		// We check first if SRL_volume is equal to volume because that will be the most common case.

#else

	// If we're not doing volume control...

		// Shift dl and dh >> 4 to convert 16 bit sample to 12 bit:

		asm volatile("swap %0" : "=r" (dl) : "0" (dl)); // SWAP high and low nybbles of dl
		asm volatile("swap %0" : "=r" (dh) : "0" (dh)); // SWAP high and low nybbles of dh		
		dl = (dh & 0XF0)|(dl & 0X0F); // Combine high nybble of dh with low nybble of dl (which is really the original low nybble of dh and the original high nybble of dl - the low nybble of dl is effectively shifted out)
		dh = dh & 0X0F; // Mask off high nybble of dh

     // Take the SS pin low to select the DAC. (Bit 4 on port B is the hardware SPI slave select pin on the Atmega1284p)

     //		PORTB &= ~0b10000; 

     // Send 16 bits to the DAC.
	// First 4 bits are configuration bits. 0011 = (DAC A, unbuffered, 1x gain, not muted)
	// Last 12 bits specify the output voltage. (MSB first)

	// The 12-bit sample is divided into two bytes, with the upper four bits in dh, and the lower eight bits in dl. 

    		//SPDR = 0b00000000 | dh;
    		//while (!(SPSR & _BV(SPIF)));
		
    		//SPDR = dl;
    		//while (!(SPSR & _BV(SPIF)));

		uint16_t sample = (dh << 8) | dl;
		mcpDacSend(sample);

 	// Do peak level detection.
		if (peakMin > sample) { peakMin = sample; }
		if (peakMax < sample) { peakMax = sample; }


#endif //DVOLUME


     // Take the SS pin high to de-select the DAC:
     // 	PORTB |= 0b10000;

}


//------------------------------------------------------------------------------
// this is the interrupt that fills the playbuffer

ISR(TIMER3_COMPB_vect) {

	//sei(); // *** TEST CODE *** enable interrupts

  // turn off calling interrupt
  TIMSK3 &= ~_BV(OCIE3B);
  
  if (sdstatus != SD_FILLING) return;

  sei(); // enable interrupts while reading the SD
  int16_t read = playing->readWaveData(sdbuff, PLAYBUFFLEN);
  cli();

  if (read > 0) { // If there was still data left in the file...
    sdend = sdbuff + read; // Set the pointer that points at the end of the buffer to the location of the start of the buffer + the number of bytes which were read.
    sdstatus = SD_READY;
  }
  else {

	if (playing->looping) { // If this file is set to loop...

		// Seek back to start of wave file:
			playing->fd->rewind();			// Seek to first byte of file.
			
			uint16_t oldVolume = playing->volume; // Store current volume level because create() will reset it.

			playing->create(*playing->fd);	

			playing->volume = oldVolume; // Restore volume level.
			playing->isplaying = 1; // Restore playing state, which is also reset by create().

			//playing->remainingBytesInChunk = 0; 	// Not sure what this is exactly but it needs to be 0 for the header skip code to work.
			//playing->readWaveData(0, 0); 		// Skip header.
		
		//playing->seek(0);

		// Read first chunk of data:
                sei(); // enable interrupts while reading the SD
                read = playing->readWaveData(sdbuff, PLAYBUFFLEN);
                cli();

  		 	sdend = sdbuff + read; // Set the pointer that points at the end of the buffer to the location of the start of the buffer + the number of bytes which were read.
  		  	sdstatus = SD_READY;
											
	} else { // File is not looping.  Cease playback on next DAC interrupt.
    		sdend = sdbuff;
    		sdstatus = SD_END_FILE;
	}

  }

}

//------------------------------------------------------------------------------
/** create an instance of WaveHC. */
WaveHC::WaveHC(void) {
  fd = 0;
}
//------------------------------------------------------------------------------
/**
 * Read a wave file's metadata and initialize member variables.
 *
 * \param[in] f A open FatReader instance for the wave file.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.  Reasons
 * for failure include I/O error, an invalid wave file or a wave
 *  file with features that WaveHC does not support.
 */
uint8_t WaveHC::create(FatReader &f) {
  // 18 byte buffer
  // can use this since Arduino and RIFF are Little Endian
  union {
    struct {
      char     id[4];
      uint32_t size;
      char     data[4];
    } riff;  // riff chunk
    struct {
      uint16_t compress;
      uint16_t channels;
      uint32_t sampleRate;
      uint32_t bytesPerSecond;
      uint16_t blockAlign;
      uint16_t bitsPerSample;
      uint16_t extraBytes;
    } fmt; // fmt data
  } buf;
  
#if OPTIMIZE_CONTIGUOUS
  // set optimized read for contiguous files
  f.optimizeContiguous();
#endif // OPTIMIZE_CONTIGUOUS

  // must start with WAVE header
  if (f.read(&buf, 12) != 12
      || strncmp(buf.riff.id, "RIFF", 4)
      || strncmp(buf.riff.data, "WAVE", 4)) {
        return false;
  }

  // next chunk must be fmt
  if (f.read(&buf, 8) != 8
      || strncmp(buf.riff.id, "fmt ", 4)) {
        return false;
  }
  
  // fmt chunk size must be 16 or 18
  uint16_t size = buf.riff.size;
  if (size == 16 || size == 18) {
    if (f.read(&buf, size) != (int16_t)size) {
      return false;
    }
  }
  else {
    // compressed data - force error
    buf.fmt.compress = 0;
  }
  
  if (buf.fmt.compress != 1 || (size == 18 && buf.fmt.extraBytes != 0)) {
    putstring_nl("Compression not supported");
    return false;
  }
  
  Channels = buf.fmt.channels;
  if (Channels > 2) {
    putstring_nl("Not mono/stereo!");
    return false;
  }
  else if (Channels > 1) {
    putstring_nl(" Warning stereo file!");
  }
  
  BitsPerSample = buf.fmt.bitsPerSample;
  if (BitsPerSample > 16) {
    putstring_nl("More than 16 bits per sample!");
    return false;
  }
  
  dwSamplesPerSec = buf.fmt.sampleRate;
  uint32_t clockRate = dwSamplesPerSec*Channels;
  uint32_t byteRate = clockRate*BitsPerSample/8;
  
#if RATE_ERROR_LEVEL > 0
  if (clockRate > MAX_CLOCK_RATE
      || byteRate > MAX_BYTE_RATE) {
    putstring_nl("Sample rate too high!");
    if (RATE_ERROR_LEVEL > 1) {
      return false;
    }
  }
  else if (byteRate > 44100 && !f.isContiguous()) {
    putstring_nl("High rate fragmented file!");
    if (RATE_ERROR_LEVEL > 1) {
      return false;
    }
  }
#endif // RATE_ERROR_LEVEL > 0

  fd = &f;

  errors = 0;
  isplaying = 0;
  remainingBytesInChunk = 0;
  
#if DVOLUME
  volume = 4095;
#endif //DVOLUME

  // position to data
  return readWaveData(0, 0) < 0 ? false: true;

}
//------------------------------------------------------------------------------
/**
 * Returns true if the player is paused else false.
 */
uint8_t WaveHC::isPaused(void) {
  cli();
  uint8_t rtn = isplaying && !(TIMSK3 & _BV(OCIE3A));
  sei();
  return rtn;
}
//------------------------------------------------------------------------------
/**
 * Pause the player.
 */
void WaveHC::pause(void) {
  cli();
  TIMSK3 &= ~_BV(OCIE3A); //disable DAC interrupt
  sei();
  fd->volume()->rawDevice()->readEnd(); // redo any partial read on resume
}
//------------------------------------------------------------------------------
/**
 * Play a wave file.
 *
 * WaveHC::create() must be called before a file can be played.
 *
 * Check the member variable WaveHC::isplaying to monitor the status
 * of the player.
 */
void WaveHC::play(uint8_t loop) {

  // setup the interrupt as necessary

  int16_t read;

  playing = this;

  // fill the play buffer
  read = readWaveData(buffer1, PLAYBUFFLEN);
  if (read <= 0) return;
  playpos = buffer1;
  playend = buffer1 + read;

  // fill the second buffer
  read = readWaveData(buffer2, PLAYBUFFLEN);
  if (read < 0) return;
  sdbuff = buffer2;
  sdend = sdbuff + read;
  sdstatus = SD_READY;
  
  // its official!
  isplaying = 1;
  looping = loop;
  
  // Setup mode for DAC ports
  mcpDacInit();
  
  // Set up timer one
  // Normal operation - no pwm not connected to pins
  TCCR3A = 0;
  // no prescaling, CTC mode
  TCCR3B = _BV(WGM12) | _BV(CS10); 
  // Sample rate - play stereo interleaved
  OCR3A =  F_CPU / (dwSamplesPerSec*Channels); // 16mhz / (22050 hz * 1 channel) = 725 clocks between interrupts
  // SD fill interrupt happens at TCNT3 == 1
  OCR3B = 1;
  // Enable timer interrupt for DAC ISR
  TIMSK3 |= _BV(OCIE3A);
  
}

//------------------------------------------------------------------------------
/** Read wave data.
 *
 * Not for use in applications.  Must be public so SD read ISR can access it.
 * Insures SD sectors are aligned with buffers.
 */
int16_t WaveHC::readWaveData(uint8_t *buff, uint16_t len) {

  if (remainingBytesInChunk == 0) {

    struct {
      char     id[4];
      uint32_t size;
    } header;

    while (1) {

      if (fd->read(&header, 8) != 8) return -1; // Read header of next chunk.

      if (!strncmp(header.id, "data", 4)) { // If this is a "data" chunk... (strncmp() returns 0 if the strings MATCH.  And 0 is false, hence the ! to invert the logic.)
        remainingBytesInChunk = header.size; // Record the size of it.
        break; // And escape from the while loop.
      }
 
      if (!fd->seekCur(header.size)) { // If this wasn't a data chunk, seek to the end of the chunk and return an error.
        return -1;
      }

    }
  }

  // make sure buffers are aligned on SD sectors

  uint16_t maxLen = PLAYBUFFLEN - fd->readPosition() % PLAYBUFFLEN;
  if (len > maxLen) len = maxLen;

  if (len > remainingBytesInChunk) {
    len = remainingBytesInChunk;
  }
  
  int16_t ret = fd->read(buff, len); // Read the data.  ret = bytes read.

  //if (ret > 0) remainingBytesInChunk -= ret;

  if (ret > 0) { 
	
	remainingBytesInChunk -= ret;

  }		
	
  return ret;

}

//------------------------------------------------------------------------------
/** Resume a paused player. */
void WaveHC::resume(void) {
  cli();
  // enable DAC interrupt
  if(isplaying) TIMSK3 |= _BV(OCIE3A);
  sei();
}

//------------------------------------------------------------------------------
/**
 * Reposition a wave file.
 *
 * \param[in] pos seek will attempt to position the file near \a pos.
 * \a pos is the byte number from the beginning of file.
 */
void WaveHC::seek(uint32_t pos) {
  // make sure buffer fill interrupt doesn't happen
  cli();
  if (fd) {
    pos -= pos % PLAYBUFFLEN;
    if (pos < PLAYBUFFLEN) pos = PLAYBUFFLEN; //don't play metadata
    uint32_t maxPos = fd->readPosition() + remainingBytesInChunk;
    if (maxPos > fd->fileSize()) maxPos = fd->fileSize();
    if (pos > maxPos) pos = maxPos;
    if (fd->seekSet(pos)) {
      // assumes a lot about the wave file
      remainingBytesInChunk = maxPos - pos;
    }
  }
  sei();
}

//------------------------------------------------------------------------------
/** Set the player's sample rate.
 *
 * \param[in] samplerate The new sample rate in samples per second.
 * No checks are done on the input parameter.
 */
void WaveHC::setSampleRate(uint32_t samplerate) {
  if (samplerate < 500) samplerate = 500;
  if (samplerate > 50000) samplerate = 50000;
  // from ladayada's library.
  cli();
  while (TCNT3 != 0); // This was TCNT0, but I believe it was supposed to be TCNT1.
  OCR3A = F_CPU / samplerate;
  sei();
}

//------------------------------------------------------------------------------
/** Stop the player. */
void WaveHC::stop(void) {
  TIMSK3 &= ~_BV(OCIE3A);   // turn off interrupt
  playing->isplaying = 0;
  playing = 0;

  // Reset peak detector so subsequent calls will return 0.
  peakMin = 65535;
  peakMax = 0;

}



/* ------------------------------------------------------------------------------
Returns the maximum difference between samples since the last reading. Value returned is between 0 and 1.
*/
float WaveHC::peakDetect(void) {
  
  float rtn;
  
  cli(); // Interrupts are disabled when reading the reseting the peak values because they could be altered in the middle of the read otherwise and wrong results returned.

  //if (peakMin > peakMax) { rtn = 0; } else { rtn = (peakMax - peakMin) / 4095.0; } // (12-bit samples) If peakMin > peakMax then we haven't played a sample since the last reading.
  
  if (peakMin > peakMax) { rtn = 0; } else { rtn = (peakMax - peakMin) / 65535.0; } // (16-bit samples) If peakMin > peakMax then we haven't played a sample since the last reading.
  
  peakMin = 65535; // Reset peak detector.
  peakMax = 0;
  
  sei();

  return rtn;

}


