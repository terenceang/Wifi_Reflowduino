
enum reflow_state {
	idle,
	preheat,
	soak,
	reflow,
	cool,
	rtd_fault
};

static const char *states_name[] = {
"idle",
"preheat",
"soak",
"reflow",
"cool",
"rtd_fault" };
