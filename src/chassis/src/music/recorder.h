#ifndef __RECORDER_H__
#define __RECORDER_H__

/* Use the newer ALSA API */
#define ALSA_PCM_NEW_HW_PARAMS_API
#include <alsa/asoundlib.h>
#include <faac.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#define RATE 32000 // PCM采样频率
#define SIZE 16    // PCM量化位数
#define CHANNELS 1 // PCM声道数目

typedef struct {
    std::string dev = "hw:0,0";
    int rate = RATE;
    int format = SND_PCM_FORMAT_S16_LE;
    int channels = CHANNELS;
    snd_pcm_uframes_t frames = (snd_pcm_uframes_t)48; // 采样频率
} SoundConf;

class Recorder
{

public:
    explicit Recorder(SoundConf conf);
    ~Recorder();

    void initPcm();
    void initAAC();
    int getSize() { return size_; }
    int getFactor() { return factor_; }
    snd_pcm_uframes_t getFrames() { return sound_conf_.frames; }
    int recodePcm(char *buffer, snd_pcm_uframes_t frame); //执行录音操作：参数1:采集到的音频的存放数据，参数2：大小
    int recodeAAC(unsigned char *&bufferOut);
    int recodeAAC();

    FILE *fpOut;
    FILE *fpPcm;

private:
    snd_pcm_t *handle_ptr_;
    int factor_;
    int size_;
    faacEncHandle encoder_;
    SoundConf sound_conf_;

    int period_size_;
    int loop_times_;
    int loop_mode_;

    char buffer_[10240];

    uint64_t nInputSamples;
    int nPCMBufferSize;
    uint8_t *pbPCMBuffer;
    uint8_t *pbAACBuffer;
    uint64_t nMaxOutputBytes;

    bool bThreadRunFlag;
};

#endif // RECORDER_H
