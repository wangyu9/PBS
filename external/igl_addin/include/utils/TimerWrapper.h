// ---------------------------------------------------------------------------------//
// Copyright (c) 2013, Regents of the University of Pennsylvania                    //
// All rights reserved.                                                             //
//                                                                                  //
// Redistribution and use in source and binary forms, with or without               //
// modification, are permitted provided that the following conditions are met:      //
//     * Redistributions of source code must retain the above copyright             //
//       notice, this list of conditions and the following disclaimer.              //
//     * Redistributions in binary form must reproduce the above copyright          //
//       notice, this list of conditions and the following disclaimer in the        //
//       documentation and/or other materials provided with the distribution.       //
//     * Neither the name of the <organization> nor the                             //
//       names of its contributors may be used to endorse or promote products       //
//       derived from this software without specific prior written permission.      //
//                                                                                  //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND  //
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    //
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           //
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY               //
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES       //
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     //
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND      //
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       //
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS    //
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                     //
//                                                                                  //
// Contact Tiantian Liu (ltt1598@gmail.com) if you have any questions.              //
//----------------------------------------------------------------------------------//
#ifndef _TIMER_WRAPPER_H_
#define _TIMER_WRAPPER_H_

// TODO: change to a more accurate timer lib instead of standard C lib
#include <time.h>


//class TimerBase
//{
//public:
//	virtual void Tic() = 0;
//	virtual void Toc() = 0;
//	virtual void Pause() = 0;
//	virtual void Resume() = 0;
//	virtual double Duration() = 0;
//	virtual double Time() const = 0;
//	virtual void Set(double time) = 0;
//};

class TimerBase
{
private:
	inline virtual double Clock() const = 0;
public:
	inline TimerBase() :m_start(0), m_end(0), m_pause_start(0), m_pause_time(0) {}
	inline virtual ~TimerBase() {}

	inline void Tic() { m_start = m_end = m_pause_time = Clock(); m_pause_time = 0; }
	inline void Toc() { m_end = Clock(); }
	inline void Pause() { m_pause_start = Clock(); }
	inline void Resume() { m_pause_time += (Clock() - m_pause_start); }
	inline double Duration() { m_end = Clock(); return (m_end - m_start - m_pause_time); }

	inline double Time() const { return (Clock() - m_start - m_pause_time); }//added by wangyu

	inline void Set(double time) { m_start = Clock() - m_pause_time - time; }//added by wangyu

protected:
	clock_t m_start;
	clock_t m_end;
	clock_t m_pause_start;
	clock_t m_pause_time;
};

class TimerWrapper : public TimerBase
{
private:
	double Clock() const { return clock() * 1.0 / (double)CLOCKS_PER_SEC; }
};

class ManualTimer : public TimerBase
{
private:
	double time;
	double Clock() const { return time; }
public:
	void Set(const double t) { time = t; }
	double Get() const { return time; }
	ManualTimer():time(0){}
};

class FrameTimer: public ManualTimer
{
public:
	void Update() { ManualTimer::Set( ManualTimer::Get() + 1.0); }
};

#endif