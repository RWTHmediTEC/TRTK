#ifndef UNIT_TEST_H_3129801785
#define UNIT_TEST_H_3129801785


#include <cassert>
#include <iostream>


using std::cout;
using std::endl;


#define HEADING(text) \
    cout << endl \
         << "*****************************************************************************" \
         << endl << "Testing " #text << endl \
         << "*****************************************************************************" \
         << endl;

#define SUBHEADING(text) cout << endl << #text << endl << endl;

#define START_TEST cout << "  Testing... ";

#define STOP_TEST cout << "Done." << endl;


#endif // UNIT_TEST_H_3129801785
