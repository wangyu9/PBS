#include "CommandLineMulti.h"

static CommandLineMulti CommandLineMultiInstance;

CommandLineMulti& CommandLineMulti::GetReference()
{
	return CommandLineMultiInstance;
}