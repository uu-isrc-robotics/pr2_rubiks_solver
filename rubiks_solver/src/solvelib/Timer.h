/*
 * Timer.h
 *
 *  Created on: 23-Oct-2008
 *      Author: chris
 *      Adapted from: http://oldmill.uchicago.edu/~wilder/Code/timer/
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <iostream>
#include <iomanip>
#include <sys/time.h>

class Timer {
	friend std::ostream& operator<<(std::ostream& os, Timer& t);

private:
	timeval start_time;
	timeval now, passed;

public:
	bool running;
	// 'running' is initially false.  A timer needs to be explicitly started
	// using 'start' or 'restart'
	Timer() :
		running(false) {
	}
	double elapsed_time();
	inline timeval getPassed() {
		return passed;
	}
	void start();
	void stop();
	double check();

}; // class timer

#endif /* TIMER_H_ */
