#ifndef OLED_H
#define OLED_H

#include <stdint.h>

/* 调用：OLED_Init；写内容后主循环持续OLED_Service完成分片刷新。 */
#define OLED_ADDR                  (0x3CU) /* SA0改变时可能为0x3D。 */
#define OLED_COLUMN_OFFSET         (0U)    /* 屏幕左右错位时调整。 */
#define OLED_WIDTH                 (128U)
#define OLED_PAGE_COUNT            (8U)
#define OLED_TEXT_SIZE_SMALL       (1U)
#define OLED_TEXT_SIZE_LARGE       (2U)
#define OLED_SMALL_CHAR_WIDTH      (6U)
#define OLED_LARGE_CHAR_WIDTH      (8U)
#define OLED_SMALL_TEXT_COLUMNS    (21U)
#define OLED_LARGE_TEXT_COLUMNS    (16U)
#define OLED_SMALL_TEXT_LINES      (8U)
#define OLED_LARGE_TEXT_LINES      (4U)

#ifdef __cplusplus
extern "C" {
#endif

/* OLED初始化 */
void OLED_Init(void);

/** 分片刷新一小段 OLED 显存；主循环中持续调用。 */
void OLED_Service(void);

/* 设置OLED显存坐标，x范围0~127，y范围0~7 */
void OLED_SetPos(uint8_t x, uint8_t y);

/* 全屏填充 */
void OLED_Fill(uint8_t fill_data);

/* 清屏 */
void OLED_CLS(void);

/* 开启OLED显示 */
void OLED_ON(void);

/* 关闭OLED显示 */
void OLED_OFF(void);

/*
 * 显示一个字符
 *
 * TextSize = 1：
 *   6×8字体
 *   line范围1~8
 *   column范围1~21
 *
 * TextSize = 2：
 *   8×16字体
 *   line范围1~4
 *   column范围1~16
 */
void OLED_ShowChar(uint8_t line,
                   uint8_t column,
                   char ch,
                   uint8_t text_size);

/* 显示字符串 */
void OLED_ShowString(uint8_t line,
                     uint8_t column,
                     const char *string,
                     uint8_t text_size);

/* 显示无符号十进制整数 */
void OLED_ShowNum(uint8_t line,
                  uint8_t column,
                  uint32_t number,
                  uint8_t length,
                  uint8_t text_size);

/* 显示带符号十进制整数 */
void OLED_ShowSignedNum(uint8_t line,
                        uint8_t column,
                        int32_t number,
                        uint8_t length,
                        uint8_t text_size);

/* 显示十六进制整数 */
void OLED_ShowHexNum(uint8_t line,
                     uint8_t column,
                     uint32_t number,
                     uint8_t length,
                     uint8_t text_size);

/* 显示二进制整数 */
void OLED_ShowBinNum(uint8_t line,
                     uint8_t column,
                     uint32_t number,
                     uint8_t length,
                     uint8_t text_size);

/*
 * 显示浮点数
 *
 * length：数字总位数，不包含符号和小数点
 * fraction_length：小数位数
 *
 * 例如：
 * Number = 12.34
 * length = 4
 * fraction_length = 2
 * 显示为 +12.34
 */
void OLED_ShowFNum(uint8_t line,
                   uint8_t column,
                   float number,
                   uint8_t length,
                   uint8_t fraction_length,
                   uint8_t text_size);

/* 幂运算，保留旧工程接口 */
uint32_t oled_pow(uint8_t m, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
