// SendToRaspberry.h

#pragma once

#define DEBUG_PORT Serial
#define TIME_DELAY 5000

class Sender {
private:
public:

	void init();
	void run();
	void update();
};


