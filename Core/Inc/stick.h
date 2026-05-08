#ifndef STICK_H
#define STICK_H

#ifdef __cplusplus
#include <string>
extern "C" {
#endif

extern float left_stick_x;
extern float left_stick_y;
extern float right_stick_x;
extern float right_stick_y;

#ifdef __cplusplus
void stick_parse(const char *msg);
}
#else
void stick_parse(const char *msg);
#endif

#endif // STICK_H
