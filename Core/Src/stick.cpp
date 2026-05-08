#include "stick.h"
#include <cstdio>
#include <cstring>

float left_stick_x = 0.00f;
float left_stick_y = 0.00f;
float right_stick_x = 0.00f;
float right_stick_y = 0.00f;

extern "C" void stick_parse(const char *msg) {
  if (msg == NULL) {
    printf("[stick_parse] Error: msg is NULL\r\n");
    return;
  }

  // sscanfの戻り値をチェック（成功時は4を返すべき）
  int ret = sscanf(msg, "L:%f,%f\nR:%f,%f", &left_stick_x, &left_stick_y,
                   &right_stick_x, &right_stick_y);

  if (ret != 4) {
    // printf("[stick_parse] Parse failed: expected 4 values, got %d\r\n", ret);
    // printf("[stick_parse] Message: %s\r\n", msg);
    return;
  }

  // printf("[stick_parse] L:%.2f,%.2f R:%.2f,%.2f\r\n", left_stick_x,
  //        left_stick_y, right_stick_x, right_stick_y);
}
