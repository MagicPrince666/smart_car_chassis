#ifndef _H264FRAMEDLIVESOURCE_HH
#define _H264FRAMEDLIVESOURCE_HH

#include <FramedSource.hh>
#include <memory>

#include "video_source.h"
#include "h264_camera.h"
#include "H264_UVC_Cap.h"

class H264FramedLiveSource : public FramedSource
{
public:
    static H264FramedLiveSource *createNew(UsageEnvironment &env)
    {
        return new H264FramedLiveSource(env);
    }

    H264FramedLiveSource(UsageEnvironment &env);
    virtual ~H264FramedLiveSource();

    static void updateTime(struct timeval &p);
    void doUpdateStart();
    static void updateDataNotify(void *d) { ((H264FramedLiveSource *)d)->doUpdateDataNotify(); };
    void doUpdateDataNotify();

protected:
    virtual void doGetNextFrame();
    virtual void doStopGettingFrames();
    virtual unsigned int maxFrameSize() const;

    void GetFrameData();

    static struct timeval sPresentationTime;
    static struct timeval sdiff;

    std::shared_ptr<VideoFactory> video_stream_factory_;
    std::shared_ptr<VideoStream> h264_video_;

    static bool sbTimeUpdate;
    EventTriggerId m_eventTriggerId;
    bool bVideoFirst;
    bool m_can_get_nextframe;
    bool m_is_queue_empty;
    bool m_started;
};

#endif