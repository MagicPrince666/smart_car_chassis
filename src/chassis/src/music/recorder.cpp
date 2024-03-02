#include "recorder.h"

Recorder::Recorder(SoundConf conf)
: sound_conf_(conf)
{
    bThreadRunFlag = true;
    size_ = 1024;

    remove("Record.aac");
    // remove("Record.pcm");

    fpOut = fopen("Record.aac", "wb");
    // fpPcm = fopen("Record.pcm", "wb");

    initPcm();
    initAAC();
}

Recorder::~Recorder()
{
    bThreadRunFlag = false;
    faacEncClose(encoder_);
    snd_pcm_close(handle_ptr_);
    delete[] pbAACBuffer;
    delete[] pbPCMBuffer;
    fclose(fpOut);
    //    fclose(fpPcm);
}

void Recorder::initPcm()
{
    int rc;
    snd_pcm_hw_params_t *params;
    unsigned int val;
    int dir;

    /* open PCM device for recording (capture) */
    rc = snd_pcm_open(&handle_ptr_, "hw:0,0", SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0) {
        fprintf(stderr, "unable to open pcm device: %s\n", snd_strerror(rc));
        exit(1);
    }

    /* alloc a hardware params object */
    snd_pcm_hw_params_alloca(&params);

    /* fill it with default values */
    snd_pcm_hw_params_any(handle_ptr_, params);

    /* interleaved mode */
    snd_pcm_hw_params_set_access(handle_ptr_, params, SND_PCM_ACCESS_RW_INTERLEAVED);

    /* signed 16 bit little ending format */
    snd_pcm_hw_params_set_format(handle_ptr_, params, (snd_pcm_format_t)sound_conf_.format);

    /* two channels */
    snd_pcm_hw_params_set_channels(handle_ptr_, params, sound_conf_.channels);

    /* 44100 bits/second sampling rate (CD quality) */
    val = sound_conf_.rate;
    snd_pcm_hw_params_set_rate_near(handle_ptr_, params, &val, &dir);

    /* set period size to 32 frames */
    snd_pcm_uframes_t frames = sound_conf_.frames; //设置每个周期多少帧、一个周期就是一个完整的中断;
    snd_pcm_hw_params_set_period_size_near(handle_ptr_, params, &frames, &dir);
    printf("frames: %lu  pcm_frames:%lu \n", frames, sound_conf_.frames);

    rc = snd_pcm_hw_params(handle_ptr_, params);
    if (rc < 0) {
        fprintf(stderr, "unable to set hw params: %s\n", snd_strerror(rc));
        exit(1);
    }

    /* use a buffer large enough to hold one period */
    snd_pcm_hw_params_get_period_size(params, &frames, &dir);
    printf("frames: %lu  pcm_frames:%lu \n", frames, sound_conf_.frames);
    sound_conf_.frames = frames;

    factor_ = sound_conf_.channels;
    factor_ = factor_ * (sound_conf_.format == SND_PCM_FORMAT_S16_LE ? 2 : 1);
    // factor 每一帧的字节

    size_ = frames * factor_; //一个周期的字节数
    printf("factor_: %d  pcm_channels:%d size:%d frames:%lu\n", factor_, sound_conf_.channels, size_, frames);
}

void Recorder::initAAC()
{
    uint64_t nSampleRate = sound_conf_.rate;                                     // 采样率
    uint32_t nChannels    = sound_conf_.channels;                                 // 声道数
    uint32_t nPCMBitSize  = sound_conf_.format == SND_PCM_FORMAT_S16_LE ? 16 : 8; // 单样本位数

    faacEncConfigurationPtr pConfiguration;

    // (1) Open FAAC engine
    // nInputSamples 帧的长度
    encoder_ = faacEncOpen(nSampleRate, nChannels, &nInputSamples, &nMaxOutputBytes);
    if (encoder_ == NULL) {
        printf("[ERROR] Failed to call faacEncOpen()\n");
        return;
    }

    //帧长度 ，每一帧 * 2  (16/8)
    nPCMBufferSize = nInputSamples * nPCMBitSize / 8;
    pbPCMBuffer    = new uint8_t[nPCMBufferSize];
    pbAACBuffer    = new uint8_t[nMaxOutputBytes];

    // (2.1) Get current encoding configuration
    pConfiguration                = faacEncGetCurrentConfiguration(encoder_);
    pConfiguration->inputFormat   = FAAC_INPUT_16BIT;
    pConfiguration->aacObjectType = LOW; // LC编码

    // (2.2) Set encoding configuration
    // nRet = faacEncSetConfiguration(encoder_, pConfiguration);
    faacEncSetConfiguration(encoder_, pConfiguration);

    snd_pcm_uframes_t framesofperod = getFrames();
    int factor                      = getFactor();

    period_size_ = framesofperod * factor;
    loop_times_    = nPCMBufferSize / period_size_;
    loop_mode_     = nPCMBufferSize % period_size_;
    loop_mode_     = loop_mode_ / factor;
}

int Recorder::recodeAAC(unsigned char *&bufferOut)
{
    if (!bThreadRunFlag)
        return 0;
    // int rc = 0;
    // printf("recodeAAC 1 %d\n",looptimes);
    for (int j = 0; j < loop_times_; j++) {
        // rc = recodePcm(buffer,getFrames());
        recodePcm(buffer_, getFrames());
        memcpy(pbPCMBuffer + j * period_size_, buffer_, getSize());
    }
    loop_mode_ = 16;
    // printf("recodeAAC 2 %d\n",loopmode);

    // rc = recodePcm(buffer,loopmode);
    recodePcm(buffer_, loop_mode_); //卡在这里 loopmode = 0 导致卡住
    // printf("recodeAAC 3\n");

    memcpy(pbPCMBuffer + loop_times_ * period_size_, buffer_, loop_mode_ * getFactor());

    // fwrite(pbPCMBuffer, 1, nPCMBufferSize, fpPcm);

    int nRet = faacEncEncode(encoder_, (int *)pbPCMBuffer, nInputSamples, pbAACBuffer, nMaxOutputBytes);
    // printf("recodeAAC 4 %d\n",nRet);

    // printf("faacEncEncode nRet:%d\n ",nRet);
    fwrite(pbAACBuffer, 1, nRet, fpOut);

    bufferOut = (unsigned char *)pbAACBuffer;

    return nRet;
}

int Recorder::recodeAAC()
{
    if (!bThreadRunFlag)
        return 0;
    // int rc = 0;
    // printf("recodeAAC 1 %d\n",looptimes);
    for (int j = 0; j < loop_times_; j++) {
        // rc = recodePcm(buffer,getFrames());
        recodePcm(buffer_, getFrames());
        memcpy(pbPCMBuffer + j * period_size_, buffer_, getSize());
    }
    loop_mode_ = 16;
    // printf("recodeAAC 2 %d\n",loopmode);

    // rc = recodePcm(buffer,loopmode);
    recodePcm(buffer_, loop_mode_); //卡在这里 loopmode = 0 导致卡住
    // printf("recodeAAC 3\n");

    memcpy(pbPCMBuffer + loop_times_ * period_size_, buffer_, loop_mode_ * getFactor());

    // fwrite(pbPCMBuffer, 1, nPCMBufferSize, fpPcm);

    int nRet = faacEncEncode(encoder_, (int *)pbPCMBuffer, nInputSamples, pbAACBuffer, nMaxOutputBytes);
    // printf("recodeAAC 4 %d\n",nRet);

    // printf("faacEncEncode nRet:%d\n ",nRet);
    fwrite(pbAACBuffer, 1, nRet, fpOut);

    return nRet;
}

int Recorder::recodePcm(char *buffer, snd_pcm_uframes_t frame)
{
    // printf("Recorder::recode 1 handle:%d bufsize:%d\n",handle,bufsize);

    if (!bThreadRunFlag)
        return 0;

    int rc;
    do {
        rc = snd_pcm_readi(handle_ptr_, buffer, frame);

        if (rc == -EPIPE) {
            /* EPIPE means overrun */
            fprintf(stderr, "overrun occurred\n");
            snd_pcm_prepare(handle_ptr_);
            usleep(100);
        } else if (rc < 0) {
            buffer = NULL;
            fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
        } else if (rc != (int)frame) {
            fprintf(stderr, "short read, read %d frames\n", rc);
        }

    } while (rc <= 0);

    return rc;
}
