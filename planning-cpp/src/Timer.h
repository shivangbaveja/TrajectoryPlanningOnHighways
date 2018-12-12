#pragma once

#include <chrono> // for std::chrono functions
 
 
/*
 * This class provides an easy way of timing the code using chrono library
 */ 
class Timer
{
private:
	// Type aliases to make accessing nested type easier
	using clock_t = std::chrono::high_resolution_clock;
	using second_t = std::chrono::duration<double, std::ratio<1> >;
	
	std::chrono::time_point<clock_t> m_beg;
 
public:
	Timer(); 	
	~Timer();
	
	void reset();
	double elapsed() const;
};