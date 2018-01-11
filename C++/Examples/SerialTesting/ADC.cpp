#include <unistd.h>
#include <cstdio>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <wiringSerial.h>

using namespace std;

int main(int argc, char *argv[])
{
	int handle = serialOpen("/dev/ttyAMA0", 9600);

	while(true){
		int data = serialGetchar(handle);
		printf("data = %c\n",data);
		if(data == 49){
			serialPrintf(handle,"hello, I'm payload 4");
			}
	}

	close(handle);

	return 0;
}
