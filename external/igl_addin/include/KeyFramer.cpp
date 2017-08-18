#include "KeyFramer.h"
#include <plugins\DeformDirect.h>//TODO: replace this with ControlStruct.h when added

#include <matrix_spline.h>

//#include <unsupported/Eigen/Splines>

template<typename T>
bool load_keyframing_config(const char * config_file_name, Animation<T>& keyframes)
{
	FILE * config_file = fopen(config_file_name, "r");
	if (NULL == config_file)
	{
		fprintf(stderr, "IOError: %s could not be opened...\n", config_file_name);
		return false;
	}
	keyframes.clear();

#ifndef LINE_MAX
#  define LINE_MAX 2048
#endif

	char line[LINE_MAX];
	int line_no = 1;
	while (fgets(line, LINE_MAX, config_file) != NULL)
	{
		char name[LINE_MAX];
		double duration;
		// Read first word containing type
		if (sscanf(line, "%s %lf\n", name, &duration) == 2)
		{
			//Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> keyframe_Selection;
			T new_frame;
			if (!load_one_keyframe(name, new_frame))
			{
				fprintf(stderr,
					"Error: load_keyframing_config() cannot open %s\n",
					name);
				fclose(config_file);
				return false;
			};
			//printf("KeyFrame Selection Matrix:(%d,%d)\n",keyframe_Selection.rows(),keyframe_Selection.cols());
			//assert(keyframe_Selection.cols()==3);
			keyframes.push_back(KeyFrame<T>(new_frame, duration, LINEAR_TRANSITION));
		}
		else
		{
			fprintf(stderr,
				"Error: load_keyframing_config() on line %d should have name and duration\n",
				line_no);
			fclose(config_file);
			return false;
		}
		line_no++;
	}
	fclose(config_file);
	//key_frame_timer.Tic();
	//if(!turn_on_handle_key_frame)
	//	key_frame_timer.Pause();
	return true;
}

template<typename T>
void update_keyframing(const Animation<T>& keyframes, const MatrixCubicSpline matrixCubicSpline, double time, T& output_frame, TransitionType keyframe_transitiontype = EASE_TRANSITION)
{
	
	if (keyframe_transitiontype < CUBIC_SPLINE_TRANSITION)
	{
		// blend only between two frames:

		size_t a = 0;
		size_t b = 0;
		double f = 0;
		keyframes.get_frame(time, a, b, f);

		double ff;
		//double ff = f;
		//double ff = 2 * f*f*f - 3.0*f*f + 2 * f;// Abrupt-in, abrupt-out

		switch (keyframe_transitiontype)
		{
		case LINEAR_TRANSITION:
			ff = f;
			break;
		case EASE_TRANSITION:
			ff = -2.0*f*f*f + 3.0*f*f;//ease
			break;
		case ABRUPT_TRANSITION:
			ff = 2 * f*f*f - 3.0*f*f + 2 * f;
			break;
		case ABRUPT_IN_TRANSITION:
			ff = pow(f, double(1.0 / 3.0));
			break;
		case ABRUPT_OUT_TRANSITION:
			ff = f*f*f;
			break;
		default:
			break;
		}

		//interpolate between two frames:
		blend_between_two_frames(keyframes[a].state, keyframes[b].state, ff, output_frame);
	}
	else
	{
		//blend between more frames:
		output_frame.csState = matrixCubicSpline.interpolate(time);
	}


	//normalize the quaternions part
	for (int i = 0; i < output_frame.csState.rows() / 2; i++)
	{
		output_frame.csState.block(i, 0, 1, 4).normalize();// first half stores the quaternions, second half stores positions
	}


	//Eigen::Spline2d keyframe_spline;
}

template<typename T>
T KeyFramer<T>::Current()
{
	double time = key_frame_timer.Time() * keyframe_slow_down_factor;

	T blendframe;
	update_keyframing(keyframes, matrixCubicSpline, time, blendframe, keyframe_transitiontype);
	return blendframe;
}

template<typename T>
bool KeyFramer<T>::Load_Config(const char * config_file_name)
{
	if (load_keyframing_config(config_file_name, keyframes))
	{
		double time = 0.;
		matrixCubicSpline.clear();
		for (int i = 0; i < keyframes.size(); i++)
		{
			time += keyframes[i].duration;
			matrixCubicSpline.append(time);
			matrixCubicSpline.append_matrix(keyframes[i].state.csState);
		}

		matrixCubicSpline.solve_coeffs();
		matrixCubicSpline.build_matrix_coeffs();

		matrixCubicSpline.print_coeffs();

		Start();
		return true;
	}
	else
	{
		return false;
	}
}

template<typename T>
bool KeyFramer<T>::Start()
{
	if (keyframes.size() > 0)
	{
		key_frame_timer.Tic();
		turn_on_key_frame = true;
		return true;
	}
	else
	{
		return false;
	}
}

template<typename T>
void KeyFramer<T>::SetOnOff(const bool t)
{
	turn_on_key_frame = t;
	if (turn_on_key_frame)
	{
		if (keyframes.size() > 0)
			key_frame_timer.Resume();
	}
	else
	{
		if (keyframes.size() > 0)
			key_frame_timer.Pause();
	}
}

template<typename T>
bool KeyFramer<T>::GetOnOff() const
{
	return turn_on_key_frame;
}

template<typename T>
void KeyFramer<T>::SetTime(const double t)
{
	key_frame_timer.Set(t);
}

template<typename T>
double KeyFramer<T>::GetTime() const
{
	return key_frame_timer.Time();
}


#ifndef IGL_HEADER_ONLY
// Explicit template specialization
//template bool load_keyframing_config(const char * config_file_name, Animation<ControlStructState>& keyframes);
//template void update_keyframing(Animation<ControlStructState>& keyframes, const MatrixCubicSpline matrixCubicSpline, TimerWrapper& key_frame_timer, ControlStructState& output_frame, double slow_down_factor, TransitionType keyframe_transitiontype);
template ControlStructState KeyFramer<ControlStructState>::Current();
template bool KeyFramer<ControlStructState>::Load_Config(const char * config_file_name);
template bool KeyFramer<ControlStructState>::Start();
template void KeyFramer<ControlStructState>::SetOnOff(const bool t);
template bool KeyFramer<ControlStructState>::GetOnOff() const;
template void KeyFramer<ControlStructState>::SetTime(const double t);
template double KeyFramer<ControlStructState>::GetTime() const;

#endif
