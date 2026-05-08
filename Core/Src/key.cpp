#include "key.h"
#include <cstdio>
#include <cstring>

bool Circle = false;
bool Cross = false;
bool Square = false;
bool Triangle = false;
bool Up = false;
bool Right = false;
bool Down = false;
bool Left = false;
bool L1 = false;
bool R1 = false;
bool L2 = false;
bool R2 = false;
bool SHARE = false;
bool OPTION = false;
bool PS = false;
bool L3 = false;
bool R3 = false;

void key_puress(std::string &msg) {
  int cross, circle, square, triangle, up, down, left, right, option, share, ps,
      l1, r1, l2, r2, l3, r3;

  // 1回のスキャンで全ての値をパース
  int ret = sscanf(
      msg.c_str(),
      "Cross:%d Circle:%d Square:%d Triangle:%d Up:%d Down:%d Left:%d Right:%d "
      "Option:%d Share:%d PS:%d L1:%d R1:%d L2:%d R2:%d L3:%d R3:%d",
      &cross, &circle, &square, &triangle, &up, &down, &left, &right, &option,
      &share, &ps, &l1, &r1, &l2, &r2, &l3, &r3);

  if (ret != 17) {
    // printf("[key_puress] Parse failed: expected 17 values, got %d\r\n", ret);
    return;
  }

  // パースした値を変数に代入
  Cross = (cross != 0);
  Circle = (circle != 0);
  Square = (square != 0);
  Triangle = (triangle != 0);
  Up = (up != 0);
  Down = (down != 0);
  Left = (left != 0);
  Right = (right != 0);
  OPTION = (option != 0);
  SHARE = (share != 0);
  PS = (ps != 0);
  L1 = (l1 != 0);
  R1 = (r1 != 0);
  L2 = (l2 != 0);
  R2 = (r2 != 0);
  L3 = (l3 != 0);
  R3 = (r3 != 0);

  // printf("[key_puress] Parsed successfully\r\n");
}

// C言語版: freertos.cから呼び出される
extern "C" void key_puress_c(const char *msg) {
  if (msg == NULL) {
    printf("[key_puress_c] Error: msg is NULL\r\n");
    return;
  }

  int cross, circle, square, triangle, up, down, left, right, option, share, ps,
      l1, r1, l2, r2, l3, r3;

  // 1回のスキャンで全ての値をパース
  int ret = sscanf(
      msg,
      "Cross:%d Circle:%d Square:%d Triangle:%d Up:%d Down:%d Left:%d Right:%d "
      "Option:%d Share:%d PS:%d L1:%d R1:%d L2:%d R2:%d L3:%d R3:%d",
      &cross, &circle, &square, &triangle, &up, &down, &left, &right, &option,
      &share, &ps, &l1, &r1, &l2, &r2, &l3, &r3);

  if (ret != 17) {
    // printf("[key_puress_c] Parse failed: expected 17 values, got %d\r\n", ret);
    // printf("[key_puress_c] Message: %s\r\n", msg);
    return;
  }

  // パースした値を変数に代入
  Cross = (cross != 0);
  Circle = (circle != 0);
  Square = (square != 0);
  Triangle = (triangle != 0);
  Up = (up != 0);
  Down = (down != 0);
  Left = (left != 0);
  Right = (right != 0);
  OPTION = (option != 0);
  SHARE = (share != 0);
  PS = (ps != 0);
  L1 = (l1 != 0);
  R1 = (r1 != 0);
  L2 = (l2 != 0);
  R2 = (r2 != 0);
  L3 = (l3 != 0);
  R3 = (r3 != 0);

  // printf("[key_puress_c] Parsed successfully\r\n");
}
