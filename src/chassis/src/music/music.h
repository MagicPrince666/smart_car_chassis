#ifndef __MUSIC_H__
#define __MUSIC_H__

#include <alsa/asoundlib.h>
#include <iostream>
#include <mpg123.h>
#include <list>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

class Music
{
public:
    Music(std::string dev = "default");
    ~Music();

    /**
     * @brief 初始化声卡
     * @return true 
     * @return false 
     */
    bool Init();

    /**
     * @brief 将语音加入播放列表
     * @param name 文件名
     */
    void PushToPlayList(std::string name);

private:
    std::string alsa_dev_;
    snd_pcm_t *playback_handle_;
    snd_pcm_hw_params_t *hw_params_;
    snd_mixer_t *mixer_handle_;
    snd_mixer_selem_id_t *mixer_sid_;
    snd_mixer_elem_t *mixer_elem_;
    mpg123_handle *mpg123_handle_;
    std::thread play_thread_;
    std::condition_variable g_cv_; // 全局条件变量
    std::mutex g_mtx_;             // 全局互斥锁.

    uint8_t wavbuffer_[32768];
    int channels_;
    long rate_;
    std::list<std::string> play_list_; // 数据队列

    int64_t cur_volume_; // 单前音量
    /**
     * @brief 播放声音
     * @param name 
     * @return true 
     * @return false 
     */
    bool PlayMp3(std::string name);

    /**
     * @brief 检查声卡设备
     * @return true 声卡设备存在
     * @return false 声卡设备不存在
     */
    bool SoundCardList();

    /**
     * @brief 列表处理
     * @return true 
     * @return false 
     */
    bool HandlePlayList();

    /**
     * @brief 循环播放
     * @return int 
     */
    int PlayLoop();

    /**
     * @brief 获取音量
     * @param elem 
     * @return int64_t 
     */
    int64_t GetVolume();

    /**
     * @brief 设置音量
     * @param elem 
     * @param volume 
     */
    void SetVolume(int64_t volume);

    /**
     * @brief 初始化混音器
     * @return true 
     * @return false 
     */
    bool MixerInit();
};

#endif
