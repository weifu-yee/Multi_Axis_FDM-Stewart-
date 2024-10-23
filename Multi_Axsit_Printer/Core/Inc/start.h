/*
 * start.h
 *
 *  Created on: Oct 18, 2024
 *      Author: sunny
 */

#ifndef INC_START_H_
#define INC_START_H_

#define START_GPIO_PORT GPIOB
#define START_GPIO_PIN GPIO_PIN_6

class START{
public:
	START();
	~START();
	void init(void);
	void getStarted(void);
private:
};

extern bool started;
extern START Start;


#endif /* INC_START_H_ */
