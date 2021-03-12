#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <conio.h>
#include "portaudio.h"

#define NUM_SECONDS		            5
#define SAMPLE_RATE		            44100
#define SYMBOL_TIME                 500e-6  //bit duration in seconds (1 bit per symbol)
#define SYMBOL_RATE                 1.0/SYMBOL_TIME;
#define SAMPS_PER_SYM               SAMPLE_RATE*SYMBOL_TIME
#define INTERMEDIATE_FREQ           10000 //Hz
#define NUM_REPS_PER_COMMAND        8
#define NUM_ONES_CAP				100 //cap on number of 1's that can be entered at prompt
#define SPACE                       0x20
#define FORWARD_KEY                 0x48 //preceded by 0x00 or 0xe0 so must call getch() twice
#define REVERSE_KEY                 0x50 //preceded by 0x00 or 0xe0 so must call getch() twice
#define RIGHT_FORWARD_KEY           0x4d //preceded by 0x00 or 0xe0 so must call getch() twice
#define LEFT_FORWARD_KEY            0x4b //preceded by 0x00 or 0xe0 so must call getch() twice
#define PAUSE_DUR                   0.0113 //pause duration in seconds
#define PI			                3.14159265
#define FRAMES_PER_BUFFER	        1024
#define SUGGESTED_LATENCY			0.050 //Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;

typedef struct {
	float *full;
	int fullLength;
} commandStruct;

void InvalidInputMessage(){

	printf("\nInput must be a positive integer less than %d without trailing white space.\n\nExiting.\n", NUM_ONES_CAP);
	exit(1);
}

void GiveErrorInfo(PaError err) {

	const PaHostErrorInfo *hostErrorInfo;

	if (err != 0) {
		fprintf( stderr, "An error occured while using the portaudio stream\n" );
		fprintf( stderr, "Error number: %d\n", err );
		fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
		// Print more information about the error.
		if( err == paUnanticipatedHostError ) {
			hostErrorInfo = Pa_GetLastHostErrorInfo();
			fprintf( stderr, "Host API error = #%ld, hostApiType = %d\n", hostErrorInfo->errorCode, hostErrorInfo->hostApiType );
			fprintf( stderr, "Host API error = %s\n", hostErrorInfo->errorText );
		}
	}

}

PaError SendFullCommandToSoundCard(float *commandToSend, int commandLength, PaStream *stream){

    float buffer[FRAMES_PER_BUFFER]; //mono output buffer
    int i, j, bufferCount;
	PaError err;

	bufferCount = (int) floor(((double) commandLength)/FRAMES_PER_BUFFER); //flooring was done to mimic orginal Matlab code and will lead to command truncation
																		   //when commandLength is not an even multiple of FRAMES_PER_BUFFER.  In practice, does not
																		   //seem to noticably impact driving quality.  That said, can zero pad appropriately if desire to
																	       //ensure even divisibility here.

	for( i=0; i < bufferCount; i++ ) {
		for( j=0; j < FRAMES_PER_BUFFER; j++ ) {
			buffer[j] = commandToSend[(i*FRAMES_PER_BUFFER)+j];
		}
		// ----------------------------------------------------------------
		err = Pa_WriteStream(stream,buffer,FRAMES_PER_BUFFER);
		// ----------------------------------------------------------------
	}
	return err;
}

commandStruct PopulateFullCommand(int *syncArray, float carrierFreq, int sampsPerSym, int numCommandReps, int commandLength, int syncArrayLength){

	int repeatingUnitLength = (syncArrayLength + commandLength);
	int commandLengthBeforeUpsample = repeatingUnitLength*numCommandReps;
	int fullCommandLength = commandLengthBeforeUpsample*sampsPerSym;
	int i,j;
	int repeatingUnit[repeatingUnitLength];
	int commandBeforeUpsample[commandLengthBeforeUpsample];
	int baseband[fullCommandLength];
	float *fullCommand;
	float carrier[fullCommandLength];
	float t[fullCommandLength];
	commandStruct commandInfo;

	//populate repeating unit
	for (i = 0; i < syncArrayLength; i++) {
		repeatingUnit[i] = syncArray[i];
	}
	for (i = 0; i < commandLength; i++) {
		repeatingUnit[i+syncArrayLength] = (i+1)%2;
	}

	//populate command before upsampling
	for (i = 0; i < numCommandReps; i++) {
		for (j = 0; j < repeatingUnitLength; j++) {
			commandBeforeUpsample[(i*repeatingUnitLength)+j] = repeatingUnit[j];
		}
	}

	//populate time vector
	for (i = 0; i < fullCommandLength; i++) {
		t[i] = i*1.0/SAMPLE_RATE;
	}

	//populate carrier
	for (i = 0; i < fullCommandLength; i++){
		carrier[i] = (float) cos((double) (2*PI*carrierFreq*t[i]));
	}

	//make baseband (upsampled) signal
	for (i = 0; i < commandLengthBeforeUpsample; i++) {
		for (j = 0; j < (int) round(SAMPS_PER_SYM); j++) {
			baseband[i * (int) round(SAMPS_PER_SYM) + j] = commandBeforeUpsample[i];
		}
	}

	//make modulated signal
	fullCommand = (float *) malloc(fullCommandLength*sizeof (float));
	for (i = 0; i < fullCommandLength; i++) {
		fullCommand[i] = baseband[i]*carrier[i];
	}

	commandInfo.full = fullCommand;
	commandInfo.fullLength = fullCommandLength;
	return commandInfo;

}

int main(){

  int pressedKey = 0xff;  //initialize to a useless value (i.e., something other than 0x00, 0xe0, or 0x20, all of which have special meaning.)
  int sync[] = {1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0};
  int syncSize = (sizeof sync)/(sizeof sync[0]);
  int *pause, i, numOnesForward, numOnesReverse, numOnesRightForward, numOnesLeftForward;
  int forwardSize, reverseSize, rightForwardSize, leftForwardSize;
  int stdinCheck;
  commandStruct fullCommand;
  PaStreamParameters outputParameters;
  PaStream *stream;
  PaError err;
  err = Pa_Initialize();
  if( err != paNoError ) GiveErrorInfo(err);

  pause = malloc(sizeof(int)*round(PAUSE_DUR*SAMPLE_RATE));
  for (i = 0; i < (int) round(PAUSE_DUR*SAMPLE_RATE); i++){
	pause[i] = 0;  //generate pause
  }

  printf("\nEnter the number of ones corresponding to the FORWARD command: ");  //e.g., 10 for some cars
  if (scanf("%d",&numOnesForward) != 1 || numOnesForward <= 0 || numOnesForward > NUM_ONES_CAP) InvalidInputMessage();
  printf("\nEnter the number of ones corresponding to the REVERSE command: "); //e.g., 40 for some cars
  if (scanf("%d",&numOnesReverse) != 1 || numOnesReverse <= 0 || numOnesReverse > NUM_ONES_CAP) InvalidInputMessage();
  printf("\nEnter the number of ones corresponding to the RIGHT FORWARD command: "); //e.g., 28 for some cars
  if (scanf("%d",&numOnesRightForward) != 1 || numOnesRightForward <= 0 || numOnesRightForward > NUM_ONES_CAP) InvalidInputMessage();
  printf("\nEnter the number of ones corresponding to the LEFT FORWARD command: "); //e.g., 34 for some cars
  if (scanf("%d",&numOnesLeftForward) != 1 || numOnesLeftForward <= 0 || numOnesLeftForward > NUM_ONES_CAP) InvalidInputMessage();
  if ((stdinCheck = getchar()) != '\n' && stdinCheck != EOF) InvalidInputMessage();
  ungetc(stdinCheck,stdin);
  printf("\nUse the arrow keys for control.  Press the space bar to exit.\n");

  forwardSize = 2*numOnesForward;
  reverseSize = 2*numOnesReverse;
  rightForwardSize = 2*numOnesRightForward;
  leftForwardSize = 2*numOnesLeftForward;

  outputParameters.device = Pa_GetDefaultOutputDevice(); // default output device
  if (outputParameters.device == paNoDevice) {
	 fprintf(stderr,"Error: No default output device.\n");
	 GiveErrorInfo(err);
	}
  outputParameters.channelCount = 1;       //mono output
  outputParameters.sampleFormat = paFloat32; // 32 bit floating point output
  outputParameters.suggestedLatency = SUGGESTED_LATENCY;
  outputParameters.hostApiSpecificStreamInfo = NULL;

  err = Pa_OpenStream(
			  &stream,
			  NULL, // no input
			  &outputParameters,
			  SAMPLE_RATE,
			  FRAMES_PER_BUFFER,
			  paClipOff,      //we won't output out of range samples so don't bother clipping them
			  NULL, //no callback, use blocking API
			  NULL ); //no callback, so no callback userData
  if( err != paNoError ) GiveErrorInfo(err);

  err = Pa_StartStream(stream);
  if(err != paNoError) GiveErrorInfo(err);

  while (pressedKey != SPACE) {
		pressedKey = getch();
        if ((pressedKey == 0x00) || (pressedKey == 0xe0)){
			pressedKey = getch();
			switch (pressedKey){
				case FORWARD_KEY:
					fullCommand = PopulateFullCommand(sync,INTERMEDIATE_FREQ,(int) round(SAMPS_PER_SYM),NUM_REPS_PER_COMMAND,forwardSize,syncSize);
					break;
				case REVERSE_KEY:
					fullCommand = PopulateFullCommand(sync,INTERMEDIATE_FREQ,(int) round(SAMPS_PER_SYM),NUM_REPS_PER_COMMAND,reverseSize,syncSize);
					break;
				case RIGHT_FORWARD_KEY:
					fullCommand = PopulateFullCommand(sync,INTERMEDIATE_FREQ,(int) round(SAMPS_PER_SYM),NUM_REPS_PER_COMMAND,rightForwardSize,syncSize);
					break;
				case LEFT_FORWARD_KEY:
					fullCommand = PopulateFullCommand(sync,INTERMEDIATE_FREQ,(int) round(SAMPS_PER_SYM),NUM_REPS_PER_COMMAND,leftForwardSize,syncSize);
					break;
				default:
					break;
			}
		} else {
			fullCommand.full = (float *) pause;
			fullCommand.fullLength = (int) round(PAUSE_DUR*SAMPLE_RATE);
		}
		err = SendFullCommandToSoundCard(fullCommand.full, fullCommand.fullLength, stream);
		if(err != paNoError) GiveErrorInfo(err);
	}
	free(fullCommand.full);
	err = Pa_StopStream(stream);
	if( err != paNoError ) GiveErrorInfo(err);
    err = Pa_CloseStream(stream);
    if( err != paNoError ) GiveErrorInfo(err);
	Pa_Terminate();

  }
