#include <M5Stack.h>

TFT_eSprite Disbuff = TFT_eSprite(&M5.Lcd);
TFT_eSprite DisCoverScrollbuff = TFT_eSprite(&M5.Lcd);

void CoverScrollText(String strNext, uint32_t color)
{
    static String strLast;
    uint32_t colorLast = 0xffff;
    uint32_t bkColor16 = DisCoverScrollbuff.color565(0x22,0x22,0x22);
    DisCoverScrollbuff.setTextSize(2);
    DisCoverScrollbuff.setTextColor(Disbuff.color565(255,0,0),bkColor16);
    DisCoverScrollbuff.setTextDatum(TC_DATUM);
    DisCoverScrollbuff.fillRect(0,0,320,60,bkColor16);
    DisCoverScrollbuff.setTextColor(color);
    for( int i = 0; i < 20 ; i++ )
    {
        DisCoverScrollbuff.fillRect(0,20,320,20,bkColor16);
        DisCoverScrollbuff.setTextColor(colorLast);
        DisCoverScrollbuff.drawString(strLast,160,20 - i);
        DisCoverScrollbuff.setTextColor(color);
        DisCoverScrollbuff.drawString(strNext,160,40 - i);
        DisCoverScrollbuff.fillRect(0,0,320,20,bkColor16);
        DisCoverScrollbuff.fillRect(0,40,320,20,bkColor16);
        delay(5);
        DisCoverScrollbuff.pushSprite(0,150);
    }
    strLast = strNext;
    colorLast = color;
}

void DisplayInit()
{
  Disbuff.createSprite(320, 240);

  DisCoverScrollbuff.createSprite(320,60);
}
