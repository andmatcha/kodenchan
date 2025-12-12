/**
 * ============================================================================
 * CAN送信側サンプルコード
 * ============================================================================
 *
 * このファイルは、distribution_board_4にCAN信号を送信する側のコード例です。
 * 実際にそのまま実行するものではなく、必要な部分をコピーして使用してください。
 *
 * 受信側のCAN ID定義:
 *   - 0x100: サーボモーター制御
 *   - 0x101: DCモーター1制御
 *   - 0x102: DCモーター2制御
 */

#include "stm32f1xx_hal.h"

// 外部で定義されているCANハンドル
extern CAN_HandleTypeDef hcan;

/**
 * @brief サーボモーターへ角度指令を送信
 * @param angle_deg 角度 (0.0〜180.0度)
 *
 * 使用例:
 *   servo_send_angle(0.0);    // 0度
 *   servo_send_angle(90.0);   // 90度
 *   servo_send_angle(180.0);  // 180度
 */
void servo_send_angle(float angle_deg) {
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  uint32_t TxMailbox;

  // 角度を0〜1800の整数値に変換 (0.0〜180.0度 → 0〜1800)
  uint16_t angle_x10 = (uint16_t)(angle_deg * 10.0f);

  // 範囲チェック
  if (angle_x10 > 1800) {
    angle_x10 = 1800;
  }

  // CANヘッダ設定
  TxHeader.StdId = 0x100;                // サーボモーター用ID
  TxHeader.IDE = CAN_ID_STD;             // 標準ID
  TxHeader.RTR = CAN_RTR_DATA;           // データフレーム
  TxHeader.DLC = 2;                      // データ長2バイト
  TxHeader.TransmitGlobalTime = DISABLE; // グローバルタイムは無効

  // データ設定: [angle_high, angle_low]
  TxData[0] = (angle_x10 >> 8) & 0xFF;   // 上位バイト
  TxData[1] = angle_x10 & 0xFF;          // 下位バイト

  // CAN送信
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

/**
 * @brief DCモーター1へ制御指令を送信
 * @param direction 方向 (0=停止, 1=正転, 2=逆転, 3=ブレーキ)
 * @param duty_percent デューティ比 (0〜100%)
 *
 * 使用例:
 *   dc_motor1_send(0, 0);    // 停止
 *   dc_motor1_send(1, 50);   // 正転、50%出力
 *   dc_motor1_send(2, 75);   // 逆転、75%出力
 *   dc_motor1_send(3, 100);  // ブレーキ
 */
void board4_send(uint8_t dc1, uint8_t dc2, uint8_t servo) {
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  uint32_t TxMailbox;

  // 範囲チェック
  if (direction > 3) {
    direction = 0;  // 不正な値の場合は停止
  }
  if (duty_percent > 100) {
    duty_percent = 100;
  }

  // CANヘッダ設定
  TxHeader.StdId = 0x208;                // ボード4用ID
  TxHeader.IDE = CAN_ID_STD;             // 標準ID
  TxHeader.RTR = CAN_RTR_DATA;           // データフレーム
  TxHeader.DLC = 2;                      // データ長2バイト
  TxHeader.TransmitGlobalTime = DISABLE; // グローバルタイムは無効

  // データ設定: [direction, duty]
  TxData[0] = direction;
  TxData[1] = duty_percent;

  // CAN送信
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

/**
 * @brief DCモーター2へ制御指令を送信
 * @param direction 方向 (0=停止, 1=正転, 2=逆転, 3=ブレーキ)
 * @param duty_percent デューティ比 (0〜100%)
 *
 * 使用例:
 *   dc_motor2_send(0, 0);    // 停止
 *   dc_motor2_send(1, 30);   // 正転、30%出力
 *   dc_motor2_send(2, 100);  // 逆転、100%出力
 *   dc_motor2_send(3, 100);  // ブレーキ
 */
void dc_motor2_send(uint8_t direction, uint8_t duty_percent) {
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  uint32_t TxMailbox;

  // 範囲チェック
  if (direction > 3) {
    direction = 0;  // 不正な値の場合は停止
  }
  if (duty_percent > 100) {
    duty_percent = 100;
  }

  // CANヘッダ設定
  TxHeader.StdId = 0x102;                // DCモーター2用ID
  TxHeader.IDE = CAN_ID_STD;             // 標準ID
  TxHeader.RTR = CAN_RTR_DATA;           // データフレーム
  TxHeader.DLC = 2;                      // データ長2バイト
  TxHeader.TransmitGlobalTime = DISABLE; // グローバルタイムは無効

  // データ設定: [direction, duty]
  TxData[0] = direction;
  TxData[1] = duty_percent;

  // CAN送信
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

/**
 * ============================================================================
 * 使用例: 実際のシーケンス
 * ============================================================================
 */

/**
 * @brief シンプルな動作テスト例
 */
void example_simple_test(void) {
  // 1. サーボを90度に設定
  servo_send_angle(90.0);
  HAL_Delay(1000);

  // 2. DCモーター1を正転50%で動作
  dc_motor1_send(1, 50);  // direction=1(正転), duty=50%
  HAL_Delay(2000);

  // 3. DCモーター1を停止
  dc_motor1_send(0, 0);   // direction=0(停止)
  HAL_Delay(500);

  // 4. DCモーター2を逆転30%で動作
  dc_motor2_send(2, 30);  // direction=2(逆転), duty=30%
  HAL_Delay(2000);

  // 5. DCモーター2をブレーキ
  dc_motor2_send(3, 100); // direction=3(ブレーキ)
  HAL_Delay(500);
}

/**
 * @brief サーボモーターのスイープテスト
 */
void example_servo_sweep(void) {
  // 0度 → 180度へスイープ
  for (float angle = 0.0f; angle <= 180.0f; angle += 10.0f) {
    servo_send_angle(angle);
    HAL_Delay(100);
  }

  HAL_Delay(500);

  // 180度 → 0度へスイープ
  for (float angle = 180.0f; angle >= 0.0f; angle -= 10.0f) {
    servo_send_angle(angle);
    HAL_Delay(100);
  }
}

/**
 * @brief DCモーター加速・減速テスト
 */
void example_dc_motor_accel_decel(void) {
  // モーター1: 加速 (0% → 100%)
  for (uint8_t duty = 0; duty <= 100; duty += 10) {
    dc_motor1_send(1, duty);  // 正転
    HAL_Delay(200);
  }

  HAL_Delay(1000);

  // モーター1: 減速 (100% → 0%)
  for (uint8_t duty = 100; duty > 0; duty -= 10) {
    dc_motor1_send(1, duty);  // 正転
    HAL_Delay(200);
  }

  // 停止
  dc_motor1_send(0, 0);
}

/**
 * @brief 複数モーター同時制御例
 */
void example_multiple_motors(void) {
  // サーボを45度に設定
  servo_send_angle(45.0);
  HAL_Delay(10);  // CAN送信間隔

  // DCモーター1を正転70%
  dc_motor1_send(1, 70);
  HAL_Delay(10);

  // DCモーター2を正転70%
  dc_motor2_send(1, 70);
  HAL_Delay(2000);

  // 全モーター停止
  dc_motor1_send(0, 0);
  HAL_Delay(10);
  dc_motor2_send(0, 0);
  HAL_Delay(10);
  servo_send_angle(90.0);  // サーボを中央に戻す
}

/**
 * @brief 正転・逆転切り替えテスト
 */
void example_direction_change(void) {
  // 正転 2秒
  dc_motor1_send(1, 60);  // direction=1(正転), duty=60%
  HAL_Delay(2000);

  // 一旦停止してから逆転 (急な方向転換を避ける)
  dc_motor1_send(0, 0);   // 停止
  HAL_Delay(500);

  // 逆転 2秒
  dc_motor1_send(2, 60);  // direction=2(逆転), duty=60%
  HAL_Delay(2000);

  // ブレーキで停止
  dc_motor1_send(3, 100); // direction=3(ブレーキ)
  HAL_Delay(500);

  // 完全停止
  dc_motor1_send(0, 0);
}

/**
 * ============================================================================
 * 代表的な制御パターン早見表
 * ============================================================================
 *
 * --- サーボモーター (CAN ID: 0x100) ---
 * servo_send_angle(0.0);     // 0度
 * servo_send_angle(45.0);    // 45度
 * servo_send_angle(90.0);    // 90度 (中央)
 * servo_send_angle(135.0);   // 135度
 * servo_send_angle(180.0);   // 180度
 *
 * --- DCモーター1 (CAN ID: 0x101) ---
 * dc_motor1_send(0, 0);      // 停止
 * dc_motor1_send(1, 25);     // 正転、25%
 * dc_motor1_send(1, 50);     // 正転、50%
 * dc_motor1_send(1, 75);     // 正転、75%
 * dc_motor1_send(1, 100);    // 正転、100%
 * dc_motor1_send(2, 50);     // 逆転、50%
 * dc_motor1_send(3, 100);    // ブレーキ
 *
 * --- DCモーター2 (CAN ID: 0x102) ---
 * dc_motor2_send(0, 0);      // 停止
 * dc_motor2_send(1, 40);     // 正転、40%
 * dc_motor2_send(2, 60);     // 逆転、60%
 * dc_motor2_send(3, 100);    // ブレーキ
 *
 * ============================================================================
 */
