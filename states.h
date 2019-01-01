
enum reflow_state {
	idle,
	preheat,
	soak,
	reflow,
	cool,
	ended,
	rtd_fault
};

static const char *states_name[] = {
"idle",
"preheat",
"soak",
"reflow",
"cool",
"ended",
"rtd_fault" };
