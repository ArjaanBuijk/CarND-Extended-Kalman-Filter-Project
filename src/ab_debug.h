// To activate debug statements
#ifndef AB_DEBUG_H_
#define AB_DEBUG_H_

#include <iostream>
using namespace std;

const bool AB_DEBUG=true; 
const bool AB_WAIT=true;   // to wait each time a debug_message is given

void debug_message(string s1);
void debug_message(string s1, string s2);

#endif /* AB_DEBUG_H_ */
