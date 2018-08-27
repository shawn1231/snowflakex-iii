#include <iostream>

using namespace std;

// Declare variables, explicit types are needed
int	a = 4;
int	b = 7;

float	c = 4;
float	d = 7; 

int main()
{
	int	ans1i = b/a;
	float	ans1f = b/a;

	int	ans2i = d/c;
	float	ans2f = d/c;

	cout << "ans1 integer: " << ans1i << endl;
	cout << "ans1 float:   " << ans1f << endl;
	cout << "ans2 integer: " << ans2i << endl;
	cout << "ans2 float:   " << ans2f << endl;

	return 0;
}
