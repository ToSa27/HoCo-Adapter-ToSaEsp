#include <hoco-base.h>

#define DEBUG(fmt, ...)

bool HoCo::isInitialized = false;
bool HoCo::isConnected = false;

HoCo::~HoCo() {
}

// config json format:
// {"n":"...","d":[{...},{...},{...}]}
void HoCo::Init() {
	DEBUG("HoCo::Init");
	isInitialized = true;
}

void HoCo::Start() {
	DEBUG("HoCo::Start");
	if (!isInitialized)
		return;
}

void HoCo::Stop() {
	DEBUG("HoCo::Stop");
}
