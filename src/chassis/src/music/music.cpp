#include "music.h"
#include <endian.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

Music::Music(std::string dev)
    : alsa_dev_(dev)
{
}

Music::~Music()
{
    if (play_thread_.joinable()) {
        play_thread_.join();
    }
    if (playback_handle_) {
        snd_pcm_drain(playback_handle_);
        snd_pcm_close(playback_handle_);
        snd_pcm_hw_params_free(hw_params_);
    }
    if (mixer_handle_) {
        snd_mixer_close(mixer_handle_);
    }
}

bool Music::SoundCardList()
{
    int card   = -1;
    if (snd_card_next(&card) < 0 || card < 0) {
        printf("sound card not faud\n");
        return false;
    }

    while (card >= 0) {
        char *name;
        if (snd_card_get_name(card, &name) < 0) {
            printf("can not get card %d name\n", card);
            break;
        }
        printf("sound card %d: %s\n", card, name);
        free(name);
        if (snd_card_next(&card) < 0) {
            printf("first sound card not found\n");
            break;
        }
    }
    return true;
}

bool Music::Init()
{
    SoundCardList();

    int err, retval;
    if ((err = snd_pcm_open(&playback_handle_, alsa_dev_.c_str(), SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf(stderr, "Could not open audio device %s (%s)\n", alsa_dev_.c_str(), snd_strerror(err));
        return false;
    }

    if ((err = snd_pcm_hw_params_malloc(&hw_params_)) < 0) {
        fprintf(stderr, "Could not setup hardware (%s)\n", snd_strerror(err));
        return false;
    }

    MixerInit();

    // init mpg123, decode first chunk and set audio output according to that.
    mpg123_init();
    mpg123_handle_ = mpg123_new(NULL, &retval);
    if (!mpg123_handle_) {
        fprintf(stderr, "Unable to init libmpg123: %s\n", mpg123_plain_strerror(retval));
        return false;
    }

#ifdef DEBUG
    mpg123_param(mpg123_handle_, MPG123_VERBOSE, 2, 0); // leave it a little vebose, just because mpg123's sample told us so
#endif

    play_thread_ = std::thread([](Music *p_this) { p_this->PlayLoop(); }, this);
    return true;
}

void Music::PushToPlayList(std::string name)
{
    std::unique_lock<std::mutex> lck(g_mtx_);
    play_list_.push_back(name);
    g_cv_.notify_all(); // 唤醒所有线程.
}

bool Music::HandlePlayList()
{
    if (play_list_.empty()) {
        return false;
    }

    std::list<std::string>::iterator itList;
    for (itList = play_list_.begin(); itList != play_list_.end();) {
        // 移除元素
        printf("play music %s \n", (*itList).c_str());
        PlayMp3(*itList);
        play_list_.erase(itList++);
        break;
    }
    return true;
}

bool Music::PlayMp3(std::string name)
{
    int err, retval;
    size_t lretval;

    if (mpg123_open(mpg123_handle_, name.c_str()) != MPG123_OK) {
        fprintf(stderr, "Unable to open file %s!\n", name.c_str());
        return false;
    }

    retval = mpg123_read(mpg123_handle_, wavbuffer_, sizeof(wavbuffer_), &lretval);

    if (retval == MPG123_NEW_FORMAT) {
        if (mpg123_getformat(mpg123_handle_, &rate_, &channels_, &retval) != MPG123_OK) {
            fprintf(stderr, "Error trying to parse stream!\n");
            return false;
        }
    } else {
        fprintf(stderr, "%s No new format rules? (%s) Ayeeee!\n", __FUNCTION__, mpg123_plain_strerror(retval));
        return false;
    }

    if ((err = snd_pcm_hw_params_any(playback_handle_, hw_params_)) < 0) {
        fprintf(stderr, "Could not initialize hardware parameter structure (%s)\n", snd_strerror(err));
        return false;
    }
    if ((err = snd_pcm_hw_params_set_access(playback_handle_, hw_params_, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        fprintf(stderr, "Could not set access type (%s)\n", snd_strerror(err));
        return false;
    }
    if ((err = snd_pcm_hw_params_set_format(playback_handle_, hw_params_, SND_PCM_FORMAT_S16_LE)) < 0) {
        fprintf(stderr, "Cannot set sample format (%s)\n", snd_strerror(err));
        return false;
    }
    if ((err = snd_pcm_hw_params_set_rate_near(playback_handle_, hw_params_, (uint32_t *)&rate_, 0)) < 0) {
        fprintf(stderr, "Cannot set sample rate to %d (%s)\n", (unsigned int)rate_, snd_strerror(err));
        return false;
    }
    if ((err = snd_pcm_hw_params_set_channels(playback_handle_, hw_params_, channels_)) < 0) {
        fprintf(stderr, "Cannot set channel count to %d (%s)\n", channels_, snd_strerror(err));
        return false;
    }
    if ((err = snd_pcm_hw_params(playback_handle_, hw_params_)) < 0) {
        fprintf(stderr, "Cannot set parameters (%s)\n", snd_strerror(err));
        return false;
    }
    //  snd_pcm_hw_params_free (hw_params);

    if ((err = snd_pcm_prepare(playback_handle_)) < 0) {
        fprintf(stderr, "Could not prepare audio interface for use (%s)\n", snd_strerror(err));
        return false;
    }
    return true;
}

int Music::PlayLoop()
{
    int err, retval;
    size_t lretval;
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    while (ros::ok())
#else
    while (rclcpp::ok())
#endif
    {
        std::unique_lock<std::mutex> lck(g_mtx_);
        g_cv_.wait_for(lck, std::chrono::milliseconds(100));
        if (!HandlePlayList()) {
            continue;
        }
        // SetVolume(cur_volume_);
        while (true) {
            retval = mpg123_read(mpg123_handle_, wavbuffer_, sizeof(wavbuffer_), &lretval);

            if (retval == MPG123_NEW_FORMAT) {
                snd_pcm_drain(playback_handle_); // let it play up until here in the former frequency
                if (mpg123_getformat(mpg123_handle_, &rate_, &channels_, &retval) != MPG123_OK) {
                    fprintf(stderr, "Error while changing stream bitrate/audio format.\n");
                    return -1;
                }
                if ((err = snd_pcm_hw_params_set_rate_near(playback_handle_, hw_params_, (uint32_t *)&rate_, 0)) < 0) {
                    fprintf(stderr, "Could not set sample rate to %d (%s)\n", (int)rate_, snd_strerror(err));
                    return -2;
                }
                if ((err = snd_pcm_hw_params_set_channels(playback_handle_, hw_params_, channels_)) < 0) {
                    fprintf(stderr, "Could not set channel count to %d (%s)\n", channels_, snd_strerror(err));
                    return -3;
                }
                if ((err = snd_pcm_hw_params(playback_handle_, hw_params_)) < 0) {
                    fprintf(stderr, "Cannot set parameters (%s)\n", snd_strerror(err));
                    return -4;
                }

            } else if (retval == MPG123_OK) {
                if (lretval) {
                    snd_pcm_writei(playback_handle_, wavbuffer_, (lretval / (channels_ << 1))); // safe to assume 16 bit output always on, so just multiply the number of channels by 2 to get the sample count
                } else {
                    break;
                }
            } else if (retval == MPG123_DONE) {
                break;
            } else {
                fprintf(stderr, "%s No new format rules? (%s) Ayeeee!\n", __FUNCTION__, mpg123_plain_strerror(retval));
                return -5;
            }
        }
        // SetVolume(0);
    }
    return 0;
}

int64_t Music::GetVolume()
{
    int64_t minv, maxv, outvol;
    snd_mixer_selem_get_playback_volume_range(mixer_elem_, &minv, &maxv);
    snd_mixer_selem_get_playback_volume(mixer_elem_, SND_MIXER_SCHN_MONO, &outvol);

    return outvol;
}

void Music::SetVolume(int64_t volume)
{
    snd_mixer_selem_set_playback_volume_all(mixer_elem_, volume);
}

bool Music::MixerInit()
{
    const char *selem_name = "Master";

    snd_mixer_open(&mixer_handle_, 0);
    snd_mixer_attach(mixer_handle_, alsa_dev_.c_str());
    snd_mixer_selem_register(mixer_handle_, NULL, NULL);
    snd_mixer_load(mixer_handle_);

    snd_mixer_selem_id_alloca(&mixer_sid_);
    snd_mixer_selem_id_set_index(mixer_sid_, 0);
    snd_mixer_selem_id_set_name(mixer_sid_, selem_name);
    mixer_elem_ = snd_mixer_find_selem(mixer_handle_, mixer_sid_);

    cur_volume_ = GetVolume();
    printf("Current volume: %ld\n", cur_volume_);

    // SetVolume(0);

    return true;
}
