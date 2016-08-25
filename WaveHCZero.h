/*
 This library is a highly modified version of Ladyada's Wave Shield library.
 I have made many changes that may have introduced bugs.  
*/

#ifndef WaveHC_h
#define WaveHC_h

//#include <FatReader.h>
#include <SD.h>


/*
class FatVolume in FatReader.h = class SdVolume in SdFat.h
class FatReader friend of FatVolume
class SdFile fiend of SdVolume

So:
FatVolume = SdVolume
FatReader = SdFile?

*/

/* 

cores/arduino/cortex_handlers.c

SAMD21E and G do not have TC6 and TC7, so the only ones available are TC3, TC4, and TC5.

Servo library uses TC4 channels 0 and 1. (Also rumored M0 pro uses TC4 for millisecond timing.)
Tone library uses TC5 channel 0

Each TC has two channels which independent counters, but the frequency of the counters is shared.
When Tone is called, it sets the frequency of the counter to do what it does. 

I'm not sure why Tone does not use one of the TCC channels which appear to be designed for PWM applications like that, but regardless, it seems like the most logical thing to do 
is use the same timer as the Tone function, because we know other libraries will avoid it, and you don't need two ways to generate sound. 

WaveHC has two interrupts called by the same TC. Each uses one counter.  I believe these counters are the same as the channels on the Zero timers.

*/


// Avoid need to search for every instance of new name.
#define FatReader SdFile


/*
If nonzero, optimize the player for contiguous files.  It takes
longer to open a file but can play contiguous files at higher rates.
Disable if you need minimum latency for open.  Also see open by index.
*/
#define OPTIMIZE_CONTIGUOUS 1

/*
See DAC ISR in WaveHC.cpp.
Must be set after call to WaveHC::create().
*/
#define DVOLUME 1


/*
Set behavior for files that exceed MAX_CLOCK_RATE or MAX_BYTE_RATE.
If RATE_ERROR_LEVEL = 2, rate too high errors are fatal.
If RATE_ERROR_LEVEL = 1, rate too high errors are warnings.
If RATE_ERROR_LEVEL = 0, rate too high errors are ignored.
*/
#define RATE_ERROR_LEVEL 0


// Set the size for wave data buffers.  Must be 256 or 512.
//#if defined(__AVR_ATmega168P__) || defined(__AVR_ATmega168__)
/** Buffer length for for 168 Arduino. */
//#define PLAYBUFFLEN 256UL
//#else 
/** Buffer length for Arduinos other than 168. */
#define PLAYBUFFLEN 512UL
//#endif 

// Define max allowed SD read rate in bytes/sec.
//#if PLAYBUFFLEN == 512UL && OPTIMIZE_CONTIGUOUS
/** Maximum SD read rate for 512 byte buffer and contiguous file */
#define MAX_BYTE_RATE 88200
//#else 
/** Maximum SD read rate for 256 byte buffer or fragmented file */
//#define MAX_BYTE_RATE 44100
//#endif 

// Define maximum clock rate for DAC.
//#if !DVOLUME
// maximum DAC clock rate 
#define MAX_CLOCK_RATE 44100
//#else 
// Decreased clock rate if volume control is used 
//#define MAX_CLOCK_RATE 22050
//#endif 


extern uint8_t SPI_busy; // If this flag is set the DAC interrupt will skip the next sample.

extern uint16_t volume; // Global volume level.
extern uint16_t SRL_volume; // Slew rate limited volume. 

//------------------------------------------------------------------------------
/**
 * \class WaveHC
 * \brief Wave file player.
 *
 * Play wave files from FAT16 and FAT32 file systems
 * on SD and SDHC flash memory cards.
 *
 */
class WaveHC {

	public:

	  uint8_t Channels; // Wave file number of channels. Mono = 1, Stereo = 2 
	  uint32_t dwSamplesPerSec; // Wave file sample rate. Must be not greater than 44100/sec.
	  uint8_t BitsPerSample;  // Wave file bits per sample.  Must be 8 or 16. 
  
	  uint32_t remainingBytesInChunk; // Remaining bytes to be played in Wave file data chunk.
  
	  uint32_t errors; // Number of times data was not available from the SD in the DAC ISR

	  volatile uint8_t isplaying; // Has the value true if a wave file is playing else false.

	  uint16_t volume; // Software volume control. See DAC ISR.
	  uint8_t looping; // Loop continuously if true.		

	  FatReader* fd; // FatReader instance for current wave file.
  
	  WaveHC(void);

	  uint8_t create(FatReader &f);
	  uint32_t getSize(void) { return fd->fileSize(); } // Return the size of the WAV file 
	  uint8_t isPaused(void);
	  void pause(void);
	  void play(uint8_t loop=false);
	  int16_t readWaveData(uint8_t *buff, uint16_t len);
	  void resume(void);
	  void seek(uint32_t pos);
	  void setSampleRate(uint32_t samplerate);
	  void stop(void);
	  float peakDetect(void); // Returns a value 0..1, indicating maximum difference between samples since last reading.
  
};

#endif //WaveHC_h
