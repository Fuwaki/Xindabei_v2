#include "track.h"
#include "meg_adc.h"
#include "log.h"
void TrackInit() {
	LOG_INFO("TrackInit start");
	// TODO: add real initialization
	LOG_INFO("TrackInit done");
}
void TrackHandler() {
	static int first = 1;
	if (first) { LOG_INFO("TrackHandler loop entered"); first = 0; }
}