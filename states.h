
enum reflow_state {
	idle,
	preheat,
	soak,
	reflow,
	cool,
	save_data,
	ended,
	rtd_fault
};

static const char *states_name[] = {
"idle",
"preheating",
"soaking",
"reflow",
"cooling",
"saving data",
"ended",
"RTD fault" };

//States
reflow_state status = idle;
reflow_state previous_status;