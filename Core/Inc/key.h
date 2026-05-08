// language: cpp
#ifndef KEY_HPP
#define KEY_HPP

#ifdef __cplusplus
#include <string>
extern "C" {
#endif

extern bool Circle;
extern bool Cross;
extern bool Square;
extern bool Triangle;
extern bool Up;
extern bool Right;
extern bool Down;
extern bool Left;
extern bool L1;
extern bool R1;
extern bool L2;
extern bool R2;
extern bool SHARE;
extern bool OPTION;
extern bool PS;
extern bool L3;
extern bool R3;

#ifdef __cplusplus
void key_puress(std::string &msg);
}
#else
#ifdef __cplusplus
extern "C" {
#endif
void key_puress_c(const char *msg);
#ifdef __cplusplus
}
#endif
#endif

#endif // KEY_HPP
