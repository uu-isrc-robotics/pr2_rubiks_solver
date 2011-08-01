/*
 * Timer.cpp
 *
 *  Created on: 23-Oct-2008
 *      Author: chrisb
 *
 *      Adapted from http://oldmill.uchicago.edu/~wilder/Code/timer/
 *       - heavily changed!
 *      NOTE: Methods are not inline anymore, as now seperated from header file.
 */

#include "Timer.h"

// Return the time the timer was going between start and stop
double Timer::elapsed_time() {
	return passed.tv_sec + (double)passed.tv_usec / 1000000.0;

}

// Start a timer.  If it is already running, restart.
void Timer::start() {
	// Set timer status to running and set the start time
	running = true;
	gettimeofday(&start_time, NULL);
	passed=now=start_time;

}


// Stop the timer
void Timer::stop() {

	// Compute passed time
	running = false;
	gettimeofday(&now, NULL);
	timersub(&now, &start_time, &passed);

}

// Return current timer
double Timer::check() {
	gettimeofday(&now, NULL);
	timersub(&now, &start_time, &passed);
	return passed.tv_sec + (double)passed.tv_usec / 1000000.0;
}

//===========================================================================
// Allow timers to be printed to ostreams using the syntax 'os << t'
// for an ostream 'os' and a timer 't'.  For example, "cout << t" will
// print out the total amount of time 't' has been "running".
std::ostream& operator<<(std::ostream& os, Timer& t) {
	if (t.running) // output so far time
		os << std::setprecision(6) << std::setiosflags(std::ios::fixed)
			<<t.check() ;
	else
		os << std::setprecision(6) << std::setiosflags(std::ios::fixed)
			<<t.elapsed_time() ;
	return os;
}

//===========================================================================
