#include <iostream>
#include "ab_debug.h"

using namespace std;

void debug_wait()
{
  if (AB_WAIT) {
    cout<<'\n'<<"Press Enter to Continue>";
    cin.ignore();
  }
  else{
    cout<<'\n';
  }
}

void debug_message(string s1)
{
  cout<<'\n'<<s1;
  debug_wait();
}

void debug_message(string s1, string s2){
  cout<<'\n'<<s1<<s2;
  debug_wait();
}
