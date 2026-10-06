/**
 * @file openmv_line_follow.c
 * @brief OpenMV通过UART_openmv发送中心点的视觉循迹模块。
 */

#include "openmv_line_follow.h"
#include "../Hardware/UART0_openmv/openmv.h"

static VisionLineFollower g_openmv_follower;

void OpenMVLineFollow_Init(void)
{
    const VisionLineConfig config = {
        OPENMV_LINE_CENTER_X,
        OPENMV_LINE_BASE_SPEED,
        OPENMV_LINE_MIN_SPEED,
        OPENMV_LINE_MAX_SPEED,
        OPENMV_LINE_KP_X1000,
        OPENMV_LINE_MAX_CENTER_X,
        OPENMV_LINE_FRAME_TIMEOUT_MS
    };

    VisionLineFollower_Init(&g_openmv_follower, &config, openmv_ReadByte);
}

void OpenMVLineFollow_Start(uint32_t now_ms)
{
    VisionLineFollower_Start(&g_openmv_follower, now_ms);
}

VisionLineState OpenMVLineFollow_Update(uint32_t now_ms)
{
    return VisionLineFollower_Update(&g_openmv_follower, now_ms);
}

void OpenMVLineFollow_Stop(void)
{
    VisionLineFollower_Stop(&g_openmv_follower);
}

VisionLineState OpenMVLineFollow_GetState(void)
{
    return g_openmv_follower.state;
}

const char *OpenMVLineFollow_GetStatusText(void)
{
    return VisionLineFollower_GetStatusText(&g_openmv_follower);
}

uint16_t OpenMVLineFollow_GetCenterX(void)
{
    return g_openmv_follower.measured_center_x;
}
