//
// Created by erayeminocak on 10/18/23.
//

#ifndef LOCATOR_MAIN_H
#define LOCATOR_MAIN_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "dms.h"

#define BUFFER_SIZE 128
#define BUILDER_SIZE 24

void Log(const char *msg);
void Log2V(const char *c1, char *c2);
void Log4VT(const char *c1, char *c2, char *c3, char *c4);
bool Locator_Handle(char data, ros::Publisher &pub);
char* GetLat();
char* GetLon();

#endif //LOCATOR_MAIN_H