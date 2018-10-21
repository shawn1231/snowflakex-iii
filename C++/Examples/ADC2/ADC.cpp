#include <unistd.h>
#include <cstdio>
#include <Navio/Util.h>
#include <Navio/ADC.h>
#include <map>
#include <iostream>
#include <string>

using namespace std;

int main()
{
	map<string,int> col_name;

	col_name.insert(make_pair("altitude",0));
	col_name.insert(make_pair("lat",1));
	col_name.insert(make_pair("lng",2));

	int max_col = 3;

	string list[max_col] = {"altitude","lat","lng"};

	int temp = col_name.find("lat");

	cout << temp << endl;


//	for( int i = 0 ; i++ ; i < max_col )
//	{
//		cout << col_name.find(list[i]) << endl;
//	}


	return;
}



/*
int main(int argc, char *argv[])
{
    ADC adc{};
    adc.init();
    float results[adc.get_channel_count()] = {0.0f};

    if (check_apm()) {
        return 1;
    }

    while (true) {
        for (int i = 0; i < ARRAY_SIZE(results); i++){
            results[i] = adc.read(i);
            printf("A%d: %.4fV ", i, results[i] / 1000);
        }
        printf("\n");
        
        usleep(500000);
    }

    return 0;
}
*/
