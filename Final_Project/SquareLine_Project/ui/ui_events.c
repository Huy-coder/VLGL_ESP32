// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "ui.h"

int directionButton = 0;
int flag = 0;
void moveForward(lv_event_t * e)
{
	// Your code here
	directionButton = 1;
  flag = 1;
}

void moveLeft(lv_event_t * e)
{
	// Your code here
	directionButton = 4;
  flag = 1;
}

void moveRight(lv_event_t * e)
{
	// Your code here
	directionButton = 2;
  flag = 1;
}

void moveBackward(lv_event_t * e)
{
	// Your code here
	directionButton = 3;
  flag = 1;
}
