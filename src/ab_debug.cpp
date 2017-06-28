#include <iostream>
#include "ab_debug.h"

using namespace std;

void debug_wait()
{
	if (AB_WAIT) {
		cout<<'\n'<<"WAITING until you enter a character";
		cout<<'\n'<<'>'; 				// show a prompt to indicate we're waiting for something.
		char AB_DUM=' '; cin>>AB_DUM;
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