
typedef int BOOL;
#define TRUE 1
#define FALSE 0

#define Statsprintf MONprintf
#define WriteExceptionLog MONprintf

#define Now getTicks()

#ifdef __cplusplus
extern "C" {
#endif
void AdjustRXLevel(int Level);
void AdjustTXLevel(int Level);
void WriteDebugLog(int Level, const char * format, ...);
void SetPot(int address, unsigned int value);
unsigned int GetPot(int address);
void SetLED(int Pin, int State);
int OKtoAdjustLevel();			// in platform specific code
void StartDAC();
void stopDAC();
void TurnroundLink();
int RadioPoll();
void MONprintf(const char * format, ...);
void PollReceivedSamples();
void HostPoll();
void SaveEEPROM(int Reg, unsigned char Val);
void SendLogToHost(char * Msg, int len);
void Sleep(int mS);
void PlatformSleep(int mS);
void SerialSendData(const unsigned char * Msg, int Len);

extern int inIndex;			// ADC Buffer half being used 0 or 1
extern int RXLevel, autoRXLevel, TXLevel;
 
#ifdef __cplusplus
}
#endif

void TFTprintf(const char * format, ...);

void CAT4016(int value);

#define DAC_SAMPLES_PER_BLOCK 1200
#define ADC_SAMPLES_PER_BLOCK 1200
#define SendSize DAC_SAMPLES_PER_BLOCK

// DebugLog Severity Levels 

#define LOGEMERGENCY 0 
#define LOGALERT 1
#define LOGCRIT 2 
#define LOGERROR 3 
#define LOGWARNING 4
#define LOGNOTICE 5
#define LOGINFO 6
#define LOGDEBUG 7