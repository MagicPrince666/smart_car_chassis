#include "H264FramedLiveSource.hh"

bool emptyBufferFlag = true;

H264FramedLiveSource::H264FramedLiveSource(UsageEnvironment &env)
    : FramedSource(env)
{
    m_can_get_nextframe = true;
    m_is_queue_empty    = false;
    bVideoFirst         = true;
    m_started           = false;
    gettimeofday(&sPresentationTime, NULL);

    // 创建具体的视频流工厂
    video_stream_factory_ = std::make_shared<UvcH264Camera>();
    // 通过工厂方法创建视屏流产品
    std::shared_ptr<VideoStream> h264_video_ptr(video_stream_factory_->createVideoStream("/dev/video3", 1280, 720, 30));
    h264_video_ = h264_video_ptr;
    // std::unique_ptr<VideoFactory> video_stream_factory_(new UvcYuyvCamera);
    // std::unique_ptr<VideoStream> h264_video_(video_stream_factory_->createVideoStream());

    m_eventTriggerId = envir().taskScheduler().createEventTrigger(H264FramedLiveSource::updateDataNotify);
}

H264FramedLiveSource::~H264FramedLiveSource()
{
    envir().taskScheduler().deleteEventTrigger(m_eventTriggerId);
}

int timeval_substract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);

    result->tv_sec  = diff / 1000000;
    result->tv_usec = diff % 1000000;

    return diff < 0;
}

void timeval_add(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
    long int total = (t2->tv_usec + 1000000 * t2->tv_sec) + (t1->tv_usec + 1000000 * t1->tv_sec);

    result->tv_sec  = total / 1000000;
    result->tv_usec = total % 1000000;
}

struct timeval H264FramedLiveSource::sPresentationTime;
struct timeval H264FramedLiveSource::sdiff;
bool H264FramedLiveSource::sbTimeUpdate = false;

void H264FramedLiveSource::updateTime(struct timeval &p)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    sPresentationTime.tv_sec  = p.tv_sec;
    sPresentationTime.tv_usec = p.tv_usec;

    timeval_substract(&sdiff, &now, &sPresentationTime);
    printf("DIFF:%ld\n", sdiff);

    sbTimeUpdate = true;
}

void H264FramedLiveSource::doUpdateStart()
{
    envir().taskScheduler().triggerEvent(m_eventTriggerId, this);
}

void H264FramedLiveSource::doUpdateDataNotify()
{
    // nextTask() = envir().taskScheduler().scheduleDelayedTask(0,(TaskFunc*)FramedSource::afterGetting,this);
    afterGetting(this);
}

void H264FramedLiveSource::GetFrameData()
{
    unsigned len = h264_video_->getData(fTo, fMaxSize, fFrameSize, fNumTruncatedBytes);

    gettimeofday(&fPresentationTime, NULL);
    afterGetting(this);

    if (!m_can_get_nextframe) {
        envir().taskScheduler().unscheduleDelayedTask(nextTask());
        m_is_queue_empty = true;
    }
}

void H264FramedLiveSource::doGetNextFrame()
{
    if (!m_started) {
        m_started = true;
    }
    GetFrameData();
}

void H264FramedLiveSource::doStopGettingFrames()
{
    printf("H264FramedLiveSource STOP FRAME 1\n");

    m_can_get_nextframe = false;

    while (!m_is_queue_empty && m_started) {
        usleep(10000);
    }

    printf("H264FramedLiveSource STOP FRAME 2\n");
}

//网络包尺寸，注意尺寸不能太小，否则会崩溃
unsigned int H264FramedLiveSource::maxFrameSize() const
{
    printf("H264FramedLiveSource::maxFrameSize\n");
    return 150000;
}