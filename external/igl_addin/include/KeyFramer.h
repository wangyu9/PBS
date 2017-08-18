#ifndef KEY_FRAMER_H
#define KEY_FRAMER_H

#include <Animation.h>
#include "utils/TimerWrapper.h"
#include <matrix_spline.h>

template<typename T>
class KeyFramer{
public:

	KeyFramer(TimerBase& t): key_frame_timer(t)
	{
		turn_on_key_frame = false;
		keyframe_slow_down_factor = 1.0;
		keyframe_transitiontype = CUBIC_SPLINE_TRANSITION;
	}

	bool Load_Config(const char * config_file_name);
	bool Start();
	T Current();

	void SetTransitionType(const TransitionType t)
	{
		keyframe_transitiontype = t;
	}
	TransitionType GetTransitionType() const
	{
		return keyframe_transitiontype;
	}

	size_t Size() { return keyframes.size(); }

	void SetOnOff(const bool t);
	bool GetOnOff() const;

	void SetTime(const double t);
	double GetTime() const;

	// Public Variables
	double keyframe_slow_down_factor;

private:
	//Private Variables
	bool turn_on_key_frame;
	TimerBase& key_frame_timer;
	Animation<T> keyframes;

	MatrixCubicSpline matrixCubicSpline;
	TransitionType keyframe_transitiontype;
};

#endif /*KEY_FRAMER_H*/