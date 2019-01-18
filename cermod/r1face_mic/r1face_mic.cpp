/*
 * Copyright (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino <alberto.cardellino@iit.it>
 *          Francesco Diotalevi <francesco.diotalevi@iit.it>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

#include <stdio.h>
#include "r1face_mic.h"
#include "CircularBuffer.h"
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>


#define STEREO              2
#define HW_STEREO_CHANNELS  9
#define SAMPLING_RATE       16000
#define CHUNK_SIZE          512             // Got from HW specifications, do not change it!!
#define TOT_SAMPLES         ((HW_STEREO_CHANNELS * STEREO) * CHUNK_SIZE)

#define BUFFER_SIZE         50

// Low level driver configurations
#define MAGIC_NUM           100
#define IOCTL_SAMPLERATE    _IOW(MAGIC_NUM, 1, int )
#define IOCTL_SETBURST      _IOWR(MAGIC_NUM, 38, int )

using namespace cer::dev;
using namespace yarp::dev;


struct inputData {
    int32_t data[TOT_SAMPLES];
};


R1faceMic::R1faceMic(): PeriodicThread(0),
                        singleChannel(false),
                        selectedChannel(-1),
                        shift(8),
                        recording(false),
                        userChannelsNum(HW_STEREO_CHANNELS-1),
                        samplingRate(SAMPLING_RATE),
                        dev_fd(-1),
                        chunkSize(CHUNK_SIZE),
                        deviceFile("/dev/micif_dev"),
                        rawBuffer(nullptr)

{
    tmpData     = new inputData;
    inputBuffer = new CircularBuffer<inputData>(BUFFER_SIZE);
}

R1faceMic::~R1faceMic()
{
    if(tmpData)
        delete tmpData;
    if(inputBuffer)
        delete inputBuffer;
}

bool R1faceMic::open(yarp::os::Searchable &params)
{
    if(params.check("audioDevice") && params.find("audioDevice").isString())
        deviceFile = params.find("audioDevice").asString();

    shift = params.check("shift", yarp::os::Value(8)).asInt();

    singleChannel = params.check("channel");
    if(singleChannel)
    {
        selectedChannel = params.find("channel").asInt32();
        if( (selectedChannel <= 0) || (selectedChannel >= HW_STEREO_CHANNELS) )
        {
            yError() << "Requested channel do not exists. Available channels are from 0 to " << HW_STEREO_CHANNELS-2;
            return false;
        }
        userChannelsNum = 1;
        selectedChannel++;  // real channels nums goes from 1 to 8; 0 is skipped
    }

    if(params.check("help"))
    {
        yInfo() << "audioDevice : device file to open. [string]";
        yInfo() << "channel <n> : use ONLY channel <n>. [int]  if missing, use all channels instead (default)";
        return false;
    }

    dev_fd = ::open(deviceFile.c_str(), O_RDONLY);
    if (dev_fd < 0)
    {
        fprintf(stderr, "Error opening %s: %s\n", deviceFile.c_str(), strerror(errno));
        return false;
    }

    ioctl(dev_fd, IOCTL_SETBURST,   chunkSize);
    ioctl(dev_fd, IOCTL_SAMPLERATE, 16000);    // Actually this ioctl seems not to work... it sample at 16KHz anyway

    yDebug() << "Input configuration is " << params.toString();
    yInfo()  << "R1faceMic device opened, starting thread";
    yInfo()  << "Using channel " << selectedChannel;

    return start();
}

bool R1faceMic::close()
{
    recording = false;
    stop();
    return true;
}


bool R1faceMic::threadInit()
{
    recording = true;
    return true;
}

void R1faceMic::run()
{
    // when not recording, do nothing
    if(!recording)
    {
        yarp::os::Time::delay(0.01);
        return;
    }

    // Just acquire raw data and place them in the buffer as fast as possible
    if (::read(dev_fd, tmpData->data, CHUNK_SIZE * HW_STEREO_CHANNELS * STEREO * sizeof(int32_t)) < 0)
    {
        yError() << "R1 face microphones: error reading data from HW";
        return;
    }

    inputBuffer->write(*tmpData);
}

void R1faceMic::threadRelease()
{
    stopRecording();
}

bool R1faceMic::getSound(yarp::sig::Sound& sound)
{
    ///////////////////////////////////////////////
    // Extract channels from acquired buffer and
    // manipulate them to get meaningful information

    while(inputBuffer->size() < 5 && recording)
    {
        yarp::os::SystemClock::delaySystem(0.01);
    }

    int chunksInBuffer = inputBuffer->size();
    sound.resize(chunkSize * chunksInBuffer, userChannelsNum);
    sound.setFrequency(samplingRate);
    sound.clear();

    int16_t   tmp  =0;
    inputData tmpChunk;


    for(int chunk=0; chunk < chunksInBuffer; chunk++)
    {
        tmpChunk = inputBuffer->read();
        rawBuffer =  tmpChunk.data;

        int ch = 0;     // stereo channel, from 0 to 9
        int j;          // selector to choose right channels only. Jumps of (channels * STEREO)
                        // for all data until the chunk is over
        int sample = 0; // sample inside the chunk, goes from 0 to chunkSize

        // N.B. first channels are the left ones.
        if(singleChannel)
        {
            //  get the desired channel; only right ones are connected, so skip left ones anyway
            for (j=HW_STEREO_CHANNELS; j< chunkSize *(HW_STEREO_CHANNELS * STEREO); j+=(HW_STEREO_CHANNELS * STEREO), sample++)
            {
                tmp = rawBuffer[j+selectedChannel] >> shift;
                sound.set(tmp, sample + (chunk*chunkSize), 0);
            }
        }
        else
        {
            // Only right channels are connected to a microphone, so skip left channels
            for (j=HW_STEREO_CHANNELS; j< chunkSize *(HW_STEREO_CHANNELS * STEREO); j+=(HW_STEREO_CHANNELS * STEREO), sample++)
            {
                for (ch=1; ch<HW_STEREO_CHANNELS; ch++)     // skip the ch0, we don't like it
                {
                    tmp = rawBuffer[ch+j] >> shift;
                    sound.set(tmp, sample + (chunk*chunkSize), ch-1);
                }
            }
        }
    }
    return true;
}

bool R1faceMic::startRecording()
{
    recording = true;
    return true;
}

bool R1faceMic::stopRecording()
{
    recording = false;
    return true;
}

bool R1faceMic::startService()
{
    threadInit();
    return false;
}

bool R1faceMic::updateService()
{
    run();
    return false;
}

bool R1faceMic::stopService()
{
    threadRelease();
    return false;
}

