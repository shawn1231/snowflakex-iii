#pragma once

#include <cstddef>

class RCInput 
{
public:
    void init();
    int read(int ch);
    int channel_count;
    RCInput();
    ~RCInput();

private:
    int open_channel(int ch);

    static const size_t CHANNEL_COUNT = 6;
    int channels[CHANNEL_COUNT];
};
