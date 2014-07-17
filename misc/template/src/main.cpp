#include <iostream>

#include <TRTK/Coordinate.hpp>

#include "main.hpp"


using namespace std;
using namespace TRTK;


int main()
{
    Coordinate<double> coordinate(1, 2, 3);
    cout << coordinate << endl;
    return 0;
}
