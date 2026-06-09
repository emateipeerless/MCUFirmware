#ifndef TELEMETRY_STORE_H
#define TELEMETRY_STORE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define TELEMETRY_DEVICE_ID       123
#define TELEMETRY_CONFIG_NAME     "MK3D"
#define TELEMETRY_JSON_MAX        2048U

void Telemetry_Init(void);

bool Telemetry_PublishFast(const char *json);
bool Telemetry_PublishSlow(const char *json);

bool Telemetry_GetFast(char *dst, size_t dst_len, uint32_t *seq_out);
bool Telemetry_GetSlow(char *dst, size_t dst_len, uint32_t *seq_out);

bool Telemetry_TakeFast(char *dst, size_t dst_len, uint32_t *seq_out);
bool Telemetry_TakeSlow(char *dst, size_t dst_len, uint32_t *seq_out);
void Telemetry_AckFast(uint32_t seq);
void Telemetry_AckSlow(uint32_t seq);

bool Telemetry_HasPendingFast(void);
bool Telemetry_HasPendingSlow(void);

#endif
