//
// Created by erayeminocak on 10/18/23.
//

#ifndef LOCATOR_MAIN_H
#define LOCATOR_MAIN_H

void Log(const char *msg);
void Log2V(const char *c1, char *c2);
void Log4VT(const char *c1, char *c2, char *c3, char *c4);
void Locator_Process(char data);
char* GetLat();
char* GetLon();

#endif //LOCATOR_MAIN_H