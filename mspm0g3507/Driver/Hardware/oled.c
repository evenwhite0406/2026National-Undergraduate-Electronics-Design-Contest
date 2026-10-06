/**
 * @file oled.c
 * @brief SSD1306 OLED I2C驱动
 *
 * OLED使用独立I2C：
 *     I2C_OLED_INST
 *
 * OLED地址：
 *     0x3C，使用7位地址
 */

#include "oled.h"
#include "oled_font.h"
#include "../PeripheralTest/bsp_i2c.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>
#include <string.h>

/* SSD1306常见7位I2C地址 */

/*
 * SSD1306 128×64一般设置为0。
 *
 * 如果你的屏幕实际是SH1106，显示发生左右偏移或者完全看不到，
 * 可以尝试改成2。
 */






/*
 * 记录最后一次I2C发送状态。
 * 当前显示函数保持void接口，因此发送错误不会中断程序。
 */
static volatile int g_oled_i2c_status = BSP_I2C_OK;
static uint8_t g_oled_buffer[OLED_WIDTH * OLED_PAGE_COUNT];
static uint8_t g_oled_dirty_pages;
static uint8_t g_oled_page_version[OLED_PAGE_COUNT];
static uint8_t g_oled_flush_active;
static uint8_t g_oled_flush_page;
static uint8_t g_oled_flush_column;
static uint8_t g_oled_flush_version;
static uint8_t g_oled_cursor_x;
static uint8_t g_oled_cursor_page;


/**
 * @brief 毫秒延时
 */
static void OLED_DelayMs(uint32_t ms)
{
    while (ms-- != 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}


/**
 * @brief OLED底层I2C初始化
 *
 * SysConfig负责：
 *  - 外设时钟
 *  - SDA/SCL引脚复用
 *
 * BSP_I2C_OLED_WriteBytes()负责：
 *  - I2C_OLED_INST控制器初始化
 */
static void OLED_I2C_Init(void)
{
    /*
     * 当前不需要额外代码。
     * 第一次调用BSP_I2C_OLED_WriteBytes()时会自动初始化。
     */
}


/**
 * @brief 向OLED写入一个控制字节和一个数据字节
 *
 * @param control 0x00表示命令，0x40表示显示数据
 * @param data    命令或显示数据
 */
static int OLED_I2C_WriteByte(uint8_t control, uint8_t data)
{
    uint8_t buffer[2];

    buffer[0] = control;
    buffer[1] = data;

    g_oled_i2c_status =
        BSP_I2C_OLED_WriteBytes(OLED_ADDR, buffer, 2U);

    return g_oled_i2c_status;
}


/**
 * @brief 写OLED命令
 */
static void OLED_WriteCmd(uint8_t command)
{
    (void) OLED_I2C_WriteByte(0x00U, command);
}


/**
 * @brief 写OLED显存数据
 */
static void OLED_WriteData(uint8_t data)
{
    uint16_t index;

    if ((g_oled_cursor_x >= OLED_WIDTH) ||
        (g_oled_cursor_page >= OLED_PAGE_COUNT)) {
        return;
    }

    index = ((uint16_t) g_oled_cursor_page * OLED_WIDTH) +
            g_oled_cursor_x;
    g_oled_buffer[index] = data;
    g_oled_dirty_pages |= (uint8_t) (1U << g_oled_cursor_page);
    g_oled_page_version[g_oled_cursor_page]++;

    if (g_oled_cursor_x < (OLED_WIDTH - 1U)) {
        g_oled_cursor_x++;
    }
}


/**
 * @brief OLED初始化
 */
void OLED_Init(void)
{
    uint16_t flush_guard;

    (void) memset(g_oled_buffer, 0, sizeof(g_oled_buffer));
    (void) memset(g_oled_page_version, 0, sizeof(g_oled_page_version));
    g_oled_dirty_pages = 0U;
    g_oled_flush_active = 0U;
    g_oled_cursor_x = 0U;
    g_oled_cursor_page = 0U;

    OLED_I2C_Init();

    /*
     * OLED模块上电后等待电源稳定。
     */
    OLED_DelayMs(200U);

    /* 关闭显示 */
    OLED_WriteCmd(0xAEU);

    /* 设置显示时钟分频和振荡频率 */
    OLED_WriteCmd(0xD5U);
    OLED_WriteCmd(0x80U);

    /* 设置复用率：1/64 */
    OLED_WriteCmd(0xA8U);
    OLED_WriteCmd(0x3FU);

    /* 设置显示偏移 */
    OLED_WriteCmd(0xD3U);
    OLED_WriteCmd(0x00U);

    /* 设置显示起始行 */
    OLED_WriteCmd(0x40U);

    /* 开启内部电荷泵 */
    OLED_WriteCmd(0x8DU);
    OLED_WriteCmd(0x14U);

    /*
     * 设置内存寻址模式。
     * 0x02表示页寻址模式。
     */
    OLED_WriteCmd(0x20U);
    OLED_WriteCmd(0x02U);

    /*
     * 段重映射。
     * 0xA1通常对应常见模块的正常方向。
     */
    OLED_WriteCmd(0xA1U);

    /*
     * COM扫描方向。
     * 0xC8通常对应常见模块的正常方向。
     */
    OLED_WriteCmd(0xC8U);

    /* 设置COM引脚硬件配置 */
    OLED_WriteCmd(0xDAU);
    OLED_WriteCmd(0x12U);

    /* 设置对比度 */
    OLED_WriteCmd(0x81U);
    OLED_WriteCmd(0xCFU);

    /* 设置预充电周期 */
    OLED_WriteCmd(0xD9U);
    OLED_WriteCmd(0xF1U);

    /* 设置VCOMH取消选择电平 */
    OLED_WriteCmd(0xDBU);
    OLED_WriteCmd(0x40U);

    /* 显示内容跟随显存 */
    OLED_WriteCmd(0xA4U);

    /* 正常显示，非反色 */
    OLED_WriteCmd(0xA6U);

    /* 关闭水平滚动 */
    OLED_WriteCmd(0x2EU);

    /* 清除OLED显存 */
    OLED_CLS();

    /* Keep the panel off until its undefined power-on RAM is cleared. */
    for (flush_guard = 0U;
         (flush_guard < 256U) &&
         ((g_oled_dirty_pages != 0U) || (g_oled_flush_active != 0U));
         flush_guard++) {
        OLED_Service();
    }

    /* 开启显示 */
    OLED_WriteCmd(0xAFU);
}


/**
 * @brief 设置OLED显存位置
 *
 * @param x 列地址，范围0~127
 * @param y 页地址，范围0~7
 */
void OLED_SetPos(uint8_t x, uint8_t y)
{
    if ((x >= OLED_WIDTH) || (y >= OLED_PAGE_COUNT)) {
        return;
    }

    g_oled_cursor_x = x;
    g_oled_cursor_page = y;

#if 0
    uint8_t column = (uint8_t) (x + OLED_COLUMN_OFFSET);

    /* 设置页地址 */
    OLED_WriteCmd((uint8_t) (0xB0U | (y & 0x07U)));

    /* 设置列地址低4位 */
    OLED_WriteCmd((uint8_t) (column & 0x0FU));

    /* 设置列地址高4位 */
    OLED_WriteCmd(
        (uint8_t) (0x10U | ((column >> 4U) & 0x0FU)));
#endif
}


/**
 * @brief 全屏填充
 */
void OLED_Fill(uint8_t fill_data)
{
    uint8_t page;

    (void) memset(g_oled_buffer, fill_data, sizeof(g_oled_buffer));
    g_oled_dirty_pages = 0xFFU;
    for (page = 0U; page < OLED_PAGE_COUNT; page++) {
        g_oled_page_version[page]++;
    }
}

void OLED_Service(void)
{
    uint8_t packet[8];
    uint8_t count;
    uint8_t page;
    uint8_t column;
    uint8_t i;

    if (g_oled_flush_active == 0U) {
        if (g_oled_dirty_pages == 0U) {
            return;
        }

        for (page = 0U; page < OLED_PAGE_COUNT; page++) {
            if ((g_oled_dirty_pages & (uint8_t) (1U << page)) != 0U) {
                g_oled_flush_page = page;
                break;
            }
        }

        g_oled_flush_column = 0U;
        g_oled_flush_version = g_oled_page_version[g_oled_flush_page];
        column = OLED_COLUMN_OFFSET;
        OLED_WriteCmd((uint8_t) (0xB0U | g_oled_flush_page));
        OLED_WriteCmd((uint8_t) (column & 0x0FU));
        OLED_WriteCmd((uint8_t) (0x10U | ((column >> 4U) & 0x0FU)));
        g_oled_flush_active = 1U;
        return;
    }

    count = (uint8_t) (OLED_WIDTH - g_oled_flush_column);
    if (count > 7U) {
        count = 7U;
    }

    packet[0] = 0x40U;
    for (i = 0U; i < count; i++) {
        packet[i + 1U] = g_oled_buffer[
            ((uint16_t) g_oled_flush_page * OLED_WIDTH) +
            g_oled_flush_column + i];
    }

    g_oled_i2c_status = BSP_I2C_OLED_WriteBytes(
        OLED_ADDR, packet, (uint8_t) (count + 1U));
    if (g_oled_i2c_status != BSP_I2C_OK) {
        g_oled_flush_active = 0U;
        return;
    }

    g_oled_flush_column = (uint8_t) (g_oled_flush_column + count);
    if (g_oled_flush_column >= OLED_WIDTH) {
        if (g_oled_page_version[g_oled_flush_page] ==
            g_oled_flush_version) {
            g_oled_dirty_pages &=
                (uint8_t) ~(1U << g_oled_flush_page);
        }
        g_oled_flush_active = 0U;
    }
}


/**
 * @brief 清屏
 */
void OLED_CLS(void)
{
    OLED_Fill(0x00U);
}


/**
 * @brief 唤醒OLED
 */
void OLED_ON(void)
{
    /* 开启内部电荷泵 */
    OLED_WriteCmd(0x8DU);
    OLED_WriteCmd(0x14U);

    /* 开启显示 */
    OLED_WriteCmd(0xAFU);
}


/**
 * @brief 关闭OLED
 */
void OLED_OFF(void)
{
    /* 关闭显示 */
    OLED_WriteCmd(0xAEU);

    /* 关闭内部电荷泵 */
    OLED_WriteCmd(0x8DU);
    OLED_WriteCmd(0x10U);
}


/**
 * @brief 检查并修正ASCII字符
 */
static char OLED_CheckCharacter(char ch)
{
    /*
     * 常见OLED字库包含ASCII 0x20~0x7E。
     * 不支持的字符使用问号代替。
     */
    if ((ch < ' ') || (ch > '~')) {
        return '?';
    }

    return ch;
}


/**
 * @brief 显示一个字符
 */
void OLED_ShowChar(uint8_t line,
                   uint8_t column,
                   char ch,
                   uint8_t text_size)
{
    uint8_t x;
    uint8_t page;
    uint8_t font_index;
    uint8_t i;

    ch = OLED_CheckCharacter(ch);
    font_index = (uint8_t) (ch - ' ');

    if (text_size == OLED_TEXT_SIZE_SMALL) {
        /*
         * 小字体：
         * 每个字符6像素宽、8像素高。
         */
        if ((line == 0U) || (column == 0U)) {
            return;
        }

        /*
         * 超过第21列时自动换到下一行。
         */
        while (column > OLED_SMALL_TEXT_COLUMNS) {
            column =
                (uint8_t) (column - OLED_SMALL_TEXT_COLUMNS);
            line++;
        }

        if (line > OLED_SMALL_TEXT_LINES) {
            return;
        }

        x = (uint8_t)
            ((column - 1U) * OLED_SMALL_CHAR_WIDTH);

        page = (uint8_t) (line - 1U);

        OLED_SetPos(x, page);

        for (i = 0U; i < OLED_SMALL_CHAR_WIDTH; i++) {
            OLED_WriteData(F6x8[font_index][i]);
        }
    } else if (text_size == OLED_TEXT_SIZE_LARGE) {
        /*
         * 大字体：
         * 每个字符8像素宽、16像素高。
         */
        if ((line == 0U) || (column == 0U)) {
            return;
        }

        /*
         * 超过第16列时自动换到下一行。
         */
        while (column > OLED_LARGE_TEXT_COLUMNS) {
            column =
                (uint8_t) (column - OLED_LARGE_TEXT_COLUMNS);
            line++;
        }

        if (line > OLED_LARGE_TEXT_LINES) {
            return;
        }

        x = (uint8_t)
            ((column - 1U) * OLED_LARGE_CHAR_WIDTH);

        /*
         * 大字体每个逻辑行占两个OLED页。
         */
        page = (uint8_t) ((line - 1U) * 2U);

        /* 写字符上半部分 */
        OLED_SetPos(x, page);

        for (i = 0U; i < 8U; i++) {
            OLED_WriteData(
                F8X16[(uint16_t) font_index * 16U + i]);
        }

        /* 写字符下半部分 */
        OLED_SetPos(x, (uint8_t) (page + 1U));

        for (i = 0U; i < 8U; i++) {
            OLED_WriteData(
                F8X16[(uint16_t) font_index * 16U + i + 8U]);
        }
    }
}


/**
 * @brief 显示字符串
 */
void OLED_ShowString(uint8_t line,
                     uint8_t column,
                     const char *string,
                     uint8_t text_size)
{
    uint16_t i = 0U;

    if (string == 0) {
        return;
    }

    while (string[i] != '\0') {
        OLED_ShowChar(line,
                      (uint8_t) (column + i),
                      string[i],
                      text_size);
        i++;
    }
}


/**
 * @brief 整数幂运算
 */
uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1U;

    while (n-- != 0U) {
        result *= m;
    }

    return result;
}


/**
 * @brief 显示无符号十进制数
 */
void OLED_ShowNum(uint8_t line,
                  uint8_t column,
                  uint32_t number,
                  uint8_t length,
                  uint8_t text_size)
{
    uint8_t i;
    uint32_t divisor;
    uint8_t digit;

    if ((length == 0U) || (length > 10U)) {
        return;
    }

    for (i = 0U; i < length; i++) {
        divisor = oled_pow(
            10U,
            (uint8_t) (length - i - 1U));

        digit = (uint8_t)
            ((number / divisor) % 10U);

        OLED_ShowChar(
            line,
            (uint8_t) (column + i),
            (char) ('0' + digit),
            text_size);
    }
}


/**
 * @brief 显示带符号十进制数
 */
void OLED_ShowSignedNum(uint8_t line,
                        uint8_t column,
                        int32_t number,
                        uint8_t length,
                        uint8_t text_size)
{
    uint32_t magnitude;

    if (number >= 0) {
        OLED_ShowChar(
            line,
            column,
            '+',
            text_size);

        magnitude = (uint32_t) number;
    } else {
        OLED_ShowChar(
            line,
            column,
            '-',
            text_size);

        /*
         * 使用int64_t避免INT32_MIN取负数溢出。
         */
        magnitude =
            (uint32_t) (-(int64_t) number);
    }

    OLED_ShowNum(
        line,
        (uint8_t) (column + 1U),
        magnitude,
        length,
        text_size);
}


/**
 * @brief 显示十六进制数
 */
void OLED_ShowHexNum(uint8_t line,
                     uint8_t column,
                     uint32_t number,
                     uint8_t length,
                     uint8_t text_size)
{
    uint8_t i;
    uint8_t digit;
    uint32_t divisor;

    if ((length == 0U) || (length > 8U)) {
        return;
    }

    for (i = 0U; i < length; i++) {
        divisor = oled_pow(
            16U,
            (uint8_t) (length - i - 1U));

        digit = (uint8_t)
            ((number / divisor) % 16U);

        if (digit < 10U) {
            OLED_ShowChar(
                line,
                (uint8_t) (column + i),
                (char) ('0' + digit),
                text_size);
        } else {
            OLED_ShowChar(
                line,
                (uint8_t) (column + i),
                (char) ('A' + digit - 10U),
                text_size);
        }
    }
}


/**
 * @brief 显示二进制数
 */
void OLED_ShowBinNum(uint8_t line,
                     uint8_t column,
                     uint32_t number,
                     uint8_t length,
                     uint8_t text_size)
{
    uint8_t i;
    uint8_t bit;

    if ((length == 0U) || (length > 32U)) {
        return;
    }

    for (i = 0U; i < length; i++) {
        bit = (uint8_t)
            ((number >>
              (uint8_t) (length - i - 1U)) & 0x01U);

        OLED_ShowChar(
            line,
            (uint8_t) (column + i),
            (char) ('0' + bit),
            text_size);
    }
}


/**
 * @brief 显示浮点数
 */
void OLED_ShowFNum(uint8_t line,
                   uint8_t column,
                   float number,
                   uint8_t length,
                   uint8_t fraction_length,
                   uint8_t text_size)
{
    uint8_t i;
    uint8_t integer_length;
    uint8_t output_column;
    uint32_t scale;
    uint32_t scaled_number;
    uint32_t divisor;
    uint8_t digit;
    float positive_number;

    if ((length == 0U) ||
        (length > 9U) ||
        (fraction_length > length)) {
        return;
    }

    if (number >= 0.0f) {
        OLED_ShowChar(
            line,
            column,
            '+',
            text_size);

        positive_number = number;
    } else {
        OLED_ShowChar(
            line,
            column,
            '-',
            text_size);

        positive_number = -number;
    }

    scale = oled_pow(10U, fraction_length);

    /*
     * 加0.5实现简单四舍五入。
     */
    scaled_number =
        (uint32_t) (positive_number *
                    (float) scale + 0.5f);

    integer_length =
        (uint8_t) (length - fraction_length);

    output_column = (uint8_t) (column + 1U);

    for (i = 0U; i < length; i++) {
        /*
         * 到达小数部分前，插入小数点。
         */
        if ((fraction_length != 0U) &&
            (i == integer_length)) {
            OLED_ShowChar(
                line,
                output_column,
                '.',
                text_size);

            output_column++;
        }

        divisor = oled_pow(
            10U,
            (uint8_t) (length - i - 1U));

        digit = (uint8_t)
            ((scaled_number / divisor) % 10U);

        OLED_ShowChar(
            line,
            output_column,
            (char) ('0' + digit),
            text_size);

        output_column++;
    }
}
