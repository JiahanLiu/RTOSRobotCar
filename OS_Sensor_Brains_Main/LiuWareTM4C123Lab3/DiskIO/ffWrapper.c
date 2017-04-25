// IDPriorityQueue.c
// PriorityQueue for ThreadID's OS
// TA: Daniel Leach

#include "../LiuWareTM4C123Lab3/ffWrapper.h"
#include "../LiuWareTM4C123Lab3/ff.h"
#include "../LiuWareTM4C123Lab3/ST7735.h"
#include "../LiuWareTM4C123Lab3/LEDS.h"
#include <stdint.h>

static FATFS g_sFatFs;
FIL Handle,Handle2;
FRESULT MountFresult;
FRESULT Fresult;

void FileSystemTest(void){
	MountFresult = f_mount(&g_sFatFs, "", 0);
  if(MountFresult){
    ST7735_DrawString(0, 0, "f_mount error", ST7735_Color565(0, 0, 255));
    while(1){};
  }
  UINT successfulreads, successfulwrites;
  char c, d;
  int16_t x, y; int i; uint32_t n;
  c = 0;
  x = 0;
  y = 10;
  n = 1;    // seed
  Fresult = f_open(&Handle2, "testFile.txt", FA_CREATE_ALWAYS|FA_WRITE);
  if(Fresult){
    ST7735_DrawString(0, 0, "testFile error", ST7735_Color565(0, 0, 255));
    while(1){};
  } else{
    for(i=0; i<FILETESTSIZE; i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      c = ((n>>24)%10)+'0'; // random digit 0 to 9
      Fresult = f_write(&Handle2, &c, 1, &successfulwrites);
      if((successfulwrites != 1) || (Fresult != FR_OK)){
        ST7735_DrawString(0, 0, "f_write error", ST7735_Color565(0, 0, 255));
        while(1){};
      }
    }
    Fresult = f_close(&Handle2);
    if(Fresult){
      ST7735_DrawString(0, 0, "file2 f_close error", ST7735_Color565(0, 0, 255));
      while(1){};
    }
  }
  n = 1;  // reseed, start over to get the same sequence
  Fresult = f_open(&Handle, "testFile.txt", FA_READ);
  if(Fresult == FR_OK){
    ST7735_DrawString(0, 0, "Opened testFile.txt", ST7735_Color565(0, 0, 255));
    for(i=0; i<FILETESTSIZE; i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      d = ((n>>24)%10)+'0'; // expected random digit 0 to 9
      Fresult = f_read(&Handle, &c, 1, &successfulreads);
      if((successfulreads == 1) && (Fresult == FR_OK) && (c == d)){
        ST7735_DrawChar(x, y, c, ST7735_Color565(255, 255, 0), 0, 1);
        x = x + 6;
        if(x > 122){
          x = 0;                          // start over on the next line
          y = y + 10;
        }
        if(y > 150){
          y = 10;                         // the screen is full
        }
      } else{
        ST7735_DrawString(0, 0, "f_read error", ST7735_Color565(0, 0, 255));
        while(1){};
      }

    }
  } else{
    ST7735_DrawString(0, 0, "Error testFile.txt (  )", ST7735_Color565(255, 0, 0));
    ST7735_SetCursor(20, 0);
    ST7735_SetTextColor(ST7735_Color565(255, 0, 0));
    ST7735_OutUDec((uint32_t)Fresult);
    while(1){};
  }
  ST7735_DrawString(0, 0, "file test passed    ", ST7735_Color565(255, 255, 255));
  Fresult = f_close(&Handle);
/*  Fresult = f_open(&Handle,"out.txt", FA_CREATE_ALWAYS|FA_WRITE);
  if(Fresult == FR_OK){
    ST7735_DrawString(0, 0, "Opened out.txt", ST7735_Color565(0, 0, 255));
    c = 'A';
    x = 0;
    y = 10;
    Fresult = f_write(&Handle, &c, 1, &successfulreads);
    ST7735_DrawChar(x, y, c, ST7735_Color565(255, 255, 0), 0, 1);
    while((c <= 'Z') && (Fresult == FR_OK)){
      x = x + 6;
      c = c + 1;
      if(x > 122){
        x = 0;                          // start over on the next line
        y = y + 10;
      }
      if(y > 150){
        break;                          // the screen is full
      }
      Fresult = f_write(&Handle, &c, 1, &successfulreads);
      ST7735_DrawChar(x, y, c, ST7735_Color565(255, 255, 0), 0, 1);
    }
  } else{
    ST7735_DrawString(0, 0, "Error out.txt (  )", ST7735_Color565(255, 0, 0));
    ST7735_SetCursor(15, 0);
    ST7735_SetTextColor(ST7735_Color565(255, 0, 0));
    ST7735_OutUDec((uint32_t)Fresult);
  }*/
	LEDS = 0;
	while(1){};
}



