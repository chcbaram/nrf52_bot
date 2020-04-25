/*
 * ili9341_regs.h
 *
 *  Created on: 2020. 3. 26.
 *      Author: HanCheol Cho
 */

#ifndef SRC_COMMON_HW_INCLUDE_SSD1351_REGS_H_
#define SRC_COMMON_HW_INCLUDE_SSD1351_REGS_H_


#define TFT_WIDTH  128
#define TFT_HEIGHT 96


// These #defines are DEPRECATED but present for older code compatibility:
#define SSD1351WIDTH  128 ///< DEPRECATED screen width
#define SSD1351HEIGHT 96  ///< DEPRECATED screen height, set to 96 for 1.27"
// (NEW CODE SHOULD IGNORE THIS, USE THE CONSTRUCTORS THAT ACCEPT WIDTH
// AND HEIGHT ARGUMENTS).

#define SSD1351_CMD_SETCOLUMN      0x15 ///< See datasheet
#define SSD1351_CMD_SETROW         0x75 ///< See datasheet
#define SSD1351_CMD_WRITERAM       0x5C ///< See datasheet
#define SSD1351_CMD_READRAM        0x5D ///< Not currently used
#define SSD1351_CMD_SETREMAP       0xA0 ///< See datasheet
#define SSD1351_CMD_STARTLINE      0xA1 ///< See datasheet
#define SSD1351_CMD_DISPLAYOFFSET  0xA2 ///< See datasheet
#define SSD1351_CMD_DISPLAYALLOFF  0xA4 ///< Not currently used
#define SSD1351_CMD_DISPLAYALLON   0xA5 ///< Not currently used
#define SSD1351_CMD_NORMALDISPLAY  0xA6 ///< See datasheet
#define SSD1351_CMD_INVERTDISPLAY  0xA7 ///< See datasheet
#define SSD1351_CMD_FUNCTIONSELECT 0xAB ///< See datasheet
#define SSD1351_CMD_DISPLAYOFF     0xAE ///< See datasheet
#define SSD1351_CMD_DISPLAYON      0xAF ///< See datasheet
#define SSD1351_CMD_PRECHARGE      0xB1 ///< See datasheet
#define SSD1351_CMD_DISPLAYENHANCE 0xB2 ///< Not currently used
#define SSD1351_CMD_CLOCKDIV       0xB3 ///< See datasheet
#define SSD1351_CMD_SETVSL         0xB4 ///< See datasheet
#define SSD1351_CMD_SETGPIO        0xB5 ///< See datasheet
#define SSD1351_CMD_PRECHARGE2     0xB6 ///< See datasheet
#define SSD1351_CMD_SETGRAY        0xB8 ///< Not currently used
#define SSD1351_CMD_USELUT         0xB9 ///< Not currently used
#define SSD1351_CMD_PRECHARGELEVEL 0xBB ///< Not currently used
#define SSD1351_CMD_VCOMH          0xBE ///< See datasheet
#define SSD1351_CMD_CONTRASTABC    0xC1 ///< See datasheet
#define SSD1351_CMD_CONTRASTMASTER 0xC7 ///< See datasheet
#define SSD1351_CMD_MUXRATIO       0xCA ///< See datasheet
#define SSD1351_CMD_COMMANDLOCK    0xFD ///< See datasheet
#define SSD1351_CMD_HORIZSCROLL    0x96 ///< Not currently used
#define SSD1351_CMD_STOPSCROLL     0x9E ///< Not currently used
#define SSD1351_CMD_STARTSCROLL    0x9F ///< Not currently used


#endif
