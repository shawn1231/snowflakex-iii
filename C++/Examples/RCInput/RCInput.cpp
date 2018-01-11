#include <unistd.h>
#include <cstdio>

#include <Navio/RCInput.h>
#include "Navio/Util.h"



int main(int argc, char *argv[])
{
    RCInput rcin{};

    if (check_apm()) {
        return 1;
    }

    rcin.init();

    while (true) 
    {
        for(int i = 0 ; i < rcin.channel_count ; i++)
	{
		int period = rcin.read(i);
	        printf("%d ", period, " ");
        }
	printf("\n");
        sleep(1);
    }

    return 0;
}
