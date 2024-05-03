#ifndef ASSYNC_TIMER_H
#define ASSYNC_TIMER_H

#include <chrono>

class AssyncTimer
{
    private:
        uint64_t Millis();
        uint64_t old_time;

    public:
        void reset();
        bool delay(int ms);
        bool delayNR(int ms);
        int deltaT();
        AssyncTimer();
        ~AssyncTimer();
};

AssyncTimer::AssyncTimer()
{
    old_time = Millis();
}

AssyncTimer::~AssyncTimer()
{
}

void AssyncTimer::reset()
{
    old_time = Millis();
}

bool AssyncTimer::delay(int ms)
{
    if(int(Millis() - old_time) > ms)
    {
        reset();
        return true;
    }

    return false;
}

bool AssyncTimer::delayNR(int ms)
{
    if(int(Millis() - old_time) > ms)
    {
        return true;
    }

    return false;
}

int AssyncTimer::deltaT()
{
  return Millis() - old_time; 
}

uint64_t AssyncTimer::Millis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

#endif // ASSYNC_TIMER_H