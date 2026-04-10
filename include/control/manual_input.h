#ifndef KODENCHAN_MANUAL_INPUT_H
#define KODENCHAN_MANUAL_INPUT_H

#include "control/arm_types.h"
#include "protocol/ac_packet_v6.h"

#include <stdbool.h>
#include <stdint.h>

#define MANUAL_INPUT_TIMEOUT_MS 1000U

void manual_input_init(ManualInputSnapshot *snapshot);
void manual_input_force_neutral(ManualInputSnapshot *snapshot, uint32_t now_ms);
void manual_input_update_from_packet(ManualInputSnapshot *snapshot, const AcPacketV6 *packet, uint32_t now_ms);
void manual_input_apply_timeout(ManualInputSnapshot *snapshot, uint32_t now_ms);
bool manual_input_to_normalized(const ManualInputSnapshot *snapshot, ManualInput *input);

#endif /* KODENCHAN_MANUAL_INPUT_H */
