/**
 * @file k230_line_follow.c
 * @brief K230通过UART_OPENMV发送中心点的视觉循迹模块。
 */

#include "k230_line_follow.h"
#include "../Hardware/UART3_OPENMV/OPENMV.h"

static VisionLineFollower g_k230_follower;

void K230LineFollow_Init(void)
{
    const VisionLineConfig config = {
        K230_LINE_CENTER_X,
        K230_LINE_BASE_SPEED,
        K230_LINE_MIN_SPEED,
        K230_LINE_MAX_SPEED,
        K230_LINE_KP_X1000,
        K230_LINE_MAX_CENTER_X,
        K230_LINE_FRAME_TIMEOUT_MS
    };

    VisionLineFollower_Init(&g_k230_follower, &config, OPENMV_ReadByte);
}

void K230LineFollow_Start(uint32_t now_ms)
{
    VisionLineFollower_Start(&g_k230_follower, now_ms);
}

VisionLineState K230LineFollow_Update(uint32_t now_ms)
{
    return VisionLineFollower_Update(&g_k230_follower, now_ms);
}

void K230LineFollow_Stop(void)
{
    VisionLineFollower_Stop(&g_k230_follower);
}

VisionLineState K230LineFollow_GetState(void)
{
    return g_k230_follower.state;
}

const char *K230LineFollow_GetStatusText(void)
{
    return VisionLineFollower_GetStatusText(&g_k230_follower);
}

uint16_t K230LineFollow_GetCenterX(void)
{
    return g_k230_follower.measured_center_x;
}
