#ifndef __SPRITES_H__
#define __SPRITES_H__

/* Character bitmaps, taken from the reference boy-vs-girl fighting demo.
   32x48 sprites drawn with draw_Bmp32x48(); 128x8 full-screen win banners
   drawn with draw_LCD(). */

extern const unsigned char boy[32*48];
extern const unsigned char girl[32*48];
extern const unsigned char boydefeat[32*48];
extern const unsigned char girldefeat[32*48];
extern const unsigned char boywin[128*8];
extern const unsigned char girlwin[128*8];
extern const unsigned char splash_screen[128*8];   /* title screen: "BATTLE ARENA" /
                                                 "START" + fighter silhouette */
extern const unsigned char digit_3[128*8];
extern const unsigned char digit_2[128*8];
extern const unsigned char digit_1[128*8];
extern const unsigned char start_word[128*8];
extern const unsigned char starting_word[128*8];

#endif
