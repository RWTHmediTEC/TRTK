#ifndef UNIT_TEST_H_3129801785
#define UNIT_TEST_H_3129801785


#include <cassert>
#include <iostream>


using std::cout;
using std::endl;
using std::flush;


#define HEADING(text) \
    cout << endl \
         << "*****************************************************************************" \
         << endl << "Testing " #text << endl \
         << "*****************************************************************************" \
         << endl << flush;

#define SUBHEADING(text) cout << endl << #text << endl << endl << flush;

#define START_TEST cout << "  Testing... " << flush;

#define STOP_TEST cout << "Done." << endl << flush;


#endif // UNIT_TEST_H_3129801785
