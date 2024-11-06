/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h" // 240*320
#include "myimage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TITLE_REFRESH 1
#define INPUT_REFRESH 2
#define OUTPUT_REFRESH 4
#define ADDI_REFRESH 8
#define WARN_REFRESH 16
#define INFO_REFRESH 32
// above defs specify which area will be refreshed


/**
 * LCD layout:
 *   0-10  empty
 *  10-34    TITLEx24
 *  34-39  empty
 *  39-40  LINE
 *  40-45  empty
 *  45-57    INPUT_msgx12
 *  57-62  empty
 *  62-86    INPUT_datax24
 *  86-91  empty
 *  91-92  LINE
 *  92-97  empty
 * 97-109   OUTPUT_msgx12
 * 109-114 empty
 * 114-138   OUTOUT_datax24
 * 138-143 empty
 * 143-159   OUTPUT_msgx16
 * 159-164 empty
 * 164-165 LINE
 * 165-170 empty
 * 170-186   ADDI_msgx16
 * 186-191 empty
 * 191-215   ADDI_errx24
 * 215-220 empty
 * 220-244   ADDI_corx24
 * 244-249 empty
 * 249-250 LINE
 * 250-255 empty
 * 
 * 255-264 reserved
 * 
 * 264-276   INFO1x12
 * 276-281 empty
 * 281-293   INFO2x12
 * 293-298 empty
 * 298-310   WARNINGx12
 */

// y coords
#define TITLE_y 10
#define INPUT_msg_y 45
#define INPUT_data_y 62
#define OUTPUT_msg_y 97
#define OUTPUT_data_y 114
#define OUTPUT_msg2_y 143
#define ADDI_msg_y 170
#define ADDI_err_y 191
#define ADDI_cor_y 220
#define INFO1_y 264
#define INFO2_y 281
#define WARN_y 298

#define LINE1_y 39
#define LINE2_y 91
#define LINE3_y 164
#define LINE4_y 249

// char sizes
#define TITLE_s 24
#define INPUT_msg_s 12
#define INPUT_data_s 24
#define OUTPUT_msg_s 12
#define OUTPUT_data_s 24
#define OUTPUT_msg2_s 16
#define ADDI_msg_s 16
#define ADDI_err_s 24
#define ADDI_cor_s 24
#define INFO1_s 12
#define INFO2_s 12
#define WARN_s 12

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define get_databit(data, bit) ((((data)[(bit)>>5])>>((bit)&31)) & 1)
#define set_databit(data, bit, u) do{(data)[(bit)>>5] = (((data)[(bit)>>5] & (~(1<<((bit)&31)))) | (((u)&1)<<((bit)&31)));}while(0)
// bit operation in an uint32_t array

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum Prog_Mode mode;
enum Decode_Result dec_res;
size_t data_length = 4; // original data length
size_t true_length = 8; // encoded hamming length
uint32_t *data = NULL;  // user input
size_t data_ptr;        // user input array potision
char int2hex[20]="0123456789ABCDEF";
int T = 0; // led flash times
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// find minimum bits need for encode `l` length data
size_t find_minimum_length(size_t l);
// get the number length in decimal
size_t num_len(size_t num);
// convert uint32_t *data to char* string with length `len`, need to be freed
char *my_data2string(uint32_t *data, size_t len);
// convert uint32_t to char* as binary string, need to be freed
char *my_2binary(size_t num);
// show data string at a specific position, if data is too long, divide into two lines.
// `s` must be 24;
void show_data_string(size_t y, size_t s, char *p);
// use hamming encode data, returns a pointer to hamming code, nned to be freed
uint32_t* encode_data(const uint32_t *src);
// decode data, returns the data(not corrected) without hamming parity length `resl`, put how many errors in `dr`, put error position in `pos`, need to be freed
uint32_t* decode_data(const uint32_t *src, enum Decode_Result *dr, size_t *pos, size_t *resl, size_t *Uu);
// show a warning message at a specific position
void show_warning_msg(char *s);

void flash_led(enum Prog_Mode m);
// init main UI for hamming code
void init_display();
// refresh and show message at given `refresh_area`
void do_display(uint8_t refresh_area);

void show_cover();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  BACK_COLOR = WHITE;
  POINT_COLOR = BLACK;
  LCD_Clear(BACK_COLOR);
  show_cover();
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  data = (uint32_t*)malloc(sizeof(uint32_t) * 1);
  data[0]=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin));
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin));
    
    while(T > 0){
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      if(--T) HAL_Delay(100);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void add_input(int num){ // callback for KEY_num
  if(mode == Cover_Mode){ // do nothing at Cover mode
    return;
  }
  do_display(WARN_REFRESH); // remove previous warnings

  if(mode == Set_Mode){ // set mode only changes length
    if(num == 0 && data_length == 1){
      show_warning_msg("WARNING: length should be positive.");
      return;
    }
    if(num == 1 && data_length >= 277){
      show_warning_msg("WARNING: length should be less than 277.");
      return;
    }
    data_length += (num<<1) - 1;
    true_length = data_length + 1 + find_minimum_length(data_length);
    do_display(INFO_REFRESH);
    return;
  }

  // check for conditions
  if(!data){
    show_warning_msg("WARNING: inner error, data=NULL");
    return;
  }
  if(mode == Encode_Mode && data_ptr >= data_length){
    show_warning_msg("WARNING: exceed length. Ignore input.");
    return;
  }
  if(mode == Decode_Mode && data_ptr >= true_length){
    show_warning_msg("WARNING: exceed length. Ignore input.");
    return;
  }
  if(num != 0 && num != 1){
    show_warning_msg("WARNING: inner error, input not 0 or 1.");
    return;
  }

  set_databit(data, data_ptr, num);
  ++data_ptr;
  do_display(INPUT_REFRESH);

  // length reaches maximum, show result
  if(mode == Decode_Mode && data_ptr == true_length){
    do_display(OUTPUT_REFRESH | ADDI_REFRESH);
    data_ptr = 0;
    size_t datal = (true_length>>5) + ((true_length&31) ? 1 : 0);
    memset(data, 0, sizeof(uint32_t) * datal);
  }
  if(mode == Encode_Mode && data_ptr == data_length){
    do_display(OUTPUT_REFRESH | ADDI_REFRESH);
    data_ptr = 0;
    size_t datal = (true_length>>5) + ((true_length&31) ? 1 : 0);
    memset(data, 0, sizeof(uint32_t) * datal);
  }
}

// skip all position like 2^k, reserve for hamming parity
size_t find_next_hamming_data_pos(size_t j){
  ++j;
  while((j & (j-1)) == 0) ++j;
  return j;
}

size_t find_minimum_length(size_t l){
  size_t i = 1;
  for(;(1ll<<i)-i < l+1; ++i); // minimum `i` for 2^i-i >= len+1
  return i;
}

size_t num_len(size_t num){
  size_t i=0;
  while(num){
    num /= 10;
    ++i;
  }
  return i;
}

char *my_data2string(uint32_t *data, size_t len){
  if(true_length <= 72){ // display as binary
    char *ss = (char*)malloc(sizeof(char)*(len+1));
    for(size_t i=0;i<len;++i){
      ss[i] = get_databit(data, i) + '0';
    }
    ss[len] = 0;
    return ss;
  } else { // display as hex
    char *ss = (char*)malloc(sizeof(char)*(((len+3)>>2)+1));
    size_t lss = 0;
    for(size_t i = 0; i < len; i += 4){
      ss[lss] = int2hex[(data[i>>5]>>(i&31)) & 15];
      ++lss;
    }
    ss[lss]=0;
    return ss;
  }
}

char *my_2binary(size_t num){
  size_t ll = num, l = 0;
  while(ll){
    ll>>=1;
    ++l;
  }
  char *s = (char*)malloc(sizeof(char)*(l+1));
  memset(s, 0, sizeof(char)*(l+1));
  l = 0;
  while(num){
    s[l] = '0' + (num & 1);
    num >>= 1;
    ++l;
  }
  for(size_t i = 0; i < (l>>1); ++i){
    char j = s[i];
    s[i] = s[l-i-1];
    s[l-i-1] = j;
  }
  s[l] = 0;
  return s;
}

uint32_t* encode_data(const uint32_t *src){
  __disable_irq();
  size_t arrl = (true_length>>5) + ((true_length & 31) ? 1 : 0);
  uint32_t *dst = (uint32_t*)malloc(sizeof(uint32_t)*arrl);
  memset(dst, 0, sizeof(uint32_t)*arrl);
  
  // P: hamming parity, U: a single bit parity for 2bit detect
  size_t P = 0, j = 2, tt, U = 0;
  for(size_t i = 0; i < data_length; ++i){
    j = find_next_hamming_data_pos(j);
    if(tt = get_databit(src, i)){
      P ^= j; // see README for reason.
    }
    U ^= tt;
    set_databit(dst, j, tt); // put original data into hamming code position
  }
  for(size_t i = 1; i < true_length; i<<=1){
    set_databit(dst, i, P&1); // put hamming parity into position
    U ^= P&1;
    P>>=1;
  }
  set_databit(dst, 0, U); // put the single bit parity
  __enable_irq();
  return dst;
}

uint32_t* decode_data(const uint32_t *src, enum Decode_Result *dr, size_t *pos, size_t *resl, size_t *Uu){
  __disable_irq();
  size_t arrl = (true_length>>5) + ((true_length & 31) ? 1 : 0);
  uint32_t *res = (uint32_t*)malloc(sizeof(uint32_t)*arrl);
  memset(res, 0, sizeof(uint32_t)*arrl);
  size_t res_ptr=0;

  size_t P = 0, U = 0, tt;
  for(size_t i = 0; i < true_length; ++i){
    if(tt = get_databit(src, i)){
      P ^= i; // see README for reason.
    }
    U ^= tt;
    if((i^(i-1))!=(i<<1)-1){ // not hamming parity, put into decoded data
      set_databit(res, res_ptr, tt);
      ++res_ptr;
    }
  }
  *pos = P;
  *Uu = U;
  if(P && U && P < true_length){ // C not 0, and checked for odd errors, 1bit
    *dr = OneBit;
  } else if(P || U || P >= true_length){ // else if not correct, 2bit
    *dr = TwoBit;
  } else {
    *dr = CORRECT;
  }
  __enable_irq();
  *resl = res_ptr;
  return res;
}

void flash_led(enum Prog_Mode m){
  switch (m)
  {
  case Encode_Mode:
    T = 1;
    break;
  case Decode_Mode:
    T = 2;
    break;
  case Set_Mode:
    T = 3;
    break;

  default:
    break;
  }
}

void switch_mode(){
  __disable_irq();
  size_t datal;
  switch (mode) // after switched, input data should be cleared
  {
  case Cover_Mode:
    mode = Encode_Mode;
    init_display();
    break;
  case Encode_Mode:
    mode = Decode_Mode;
    datal = (true_length>>5) + ((true_length&31) ? 1 : 0);
    memset(data, 0, sizeof(uint32_t) * datal);
    data_ptr = 0;
    break;
  case Decode_Mode:
    mode = Set_Mode;
    datal = (true_length>>5) + ((true_length&31) ? 1 : 0);
    memset(data, 0, sizeof(uint32_t) * datal);
    data_ptr = 0;
    break;
  case Set_Mode:
    mode = Encode_Mode;
    true_length = find_minimum_length(data_length) + data_length + 1;
    free(data);
    datal = (true_length>>5) + ((true_length&31) ? 1 : 0);
    data = (uint32_t*)malloc(sizeof(uint32_t) * datal); // length changed, realloc input data buffer
    memset(data, 0, sizeof(uint32_t) * datal);
    data_ptr = 0;
    break;

  default:
    break;
  }
  // refresh all display
  do_display(TITLE_REFRESH | INPUT_REFRESH | OUTPUT_REFRESH | ADDI_REFRESH | WARN_REFRESH | INFO_REFRESH);
  __enable_irq();
  flash_led(mode);
}

void show_warning_msg(char *s){
  LCD_Fill(0, WARN_y, 240, WARN_y+WARN_s, BACK_COLOR);
  LCD_ShowString_trans(10, WARN_y, 220, WARN_s, WARN_s, s);
}

void show_data_string(size_t y, size_t s, char *p){
  size_t l = strlen(p);
  if(l<=18){
    LCD_ShowString_trans(10, y, 220, s, s, p);
  } else if(l<=36){
    LCD_ShowString_trans(10, y, 220, s>>1, s>>1, p);
  } else {
    char pt = p[36];
    p[36] = 0;
    LCD_ShowString_trans(10, y-1, 220, s>>1, s>>1, p);
    p[36] = pt;
    LCD_ShowString_trans(10, y+(s>>1)+1, 220, s>>1, s>>1, p+36);
  }
}

void init_display(){
  LCD_Clear(BACK_COLOR);
  LCD_DrawLine(10, LINE1_y, 230, LINE1_y);
  LCD_DrawLine(10, LINE2_y, 230, LINE2_y);
  LCD_DrawLine(10, LINE3_y, 230, LINE3_y);
  LCD_DrawLine(10, LINE4_y, 230, LINE4_y);

  LCD_ShowString_trans(10, INPUT_msg_y, 220, INPUT_msg_y, INPUT_msg_s, "Original data:");
  LCD_ShowString_trans(10, OUTPUT_msg_y, 220, OUTPUT_msg_y, OUTPUT_msg_s, "Hamming code:");

  // above are the things that will not change during execution

  LCD_Fill(0, OUTPUT_msg2_y, 240, OUTPUT_msg2_y+OUTPUT_msg2_s, BACK_COLOR);
  size_t numl = num_len(true_length-data_length);
  LCD_ShowString_trans(10, OUTPUT_msg2_y+4, 220, 12, 12, "with ");
  LCD_ShowNum(10+ 5*6, OUTPUT_msg2_y, true_length-data_length, numl, OUTPUT_msg2_s);
  LCD_ShowString_trans(10 + 5*6 + numl*8, OUTPUT_msg2_y+4, 220, 12, 12, " parity bits.");

  do_display(TITLE_REFRESH);
  do_display(INFO_REFRESH);
}

void do_display(uint8_t refresh_area){
  if(mode == Cover_Mode) return;
  if(refresh_area & TITLE_REFRESH){ // repaint title
    refresh_area ^= TITLE_REFRESH;
    LCD_Fill(0, TITLE_y, 240, TITLE_y+TITLE_s, BACK_COLOR);
    switch (mode)
    {
    case Encode_Mode:
      LCD_ShowString_trans(10, TITLE_y, 220, TITLE_s, TITLE_s, "Encode Mode");
      break;
    case Decode_Mode:
      LCD_ShowString_trans(10, TITLE_y, 220, TITLE_s, TITLE_s, "Decode Mode");
      break;
    case Set_Mode:
      LCD_ShowString_trans(10, TITLE_y, 220, TITLE_s, TITLE_s, "Length Set Mode");
      break;
    
    default:
      break;
    }
  }

  if(refresh_area & INFO_REFRESH){ // repaint info (data len: xxx)
    refresh_area ^= INFO_REFRESH;
    LCD_Fill(0, INFO1_y, 240, INFO1_y+INFO1_s, BACK_COLOR);
    LCD_Fill(0, INFO2_y, 240, INFO2_y+INFO2_s, BACK_COLOR);
    /**
     * data len: xxx
     * hamm len: xxx
     */
    LCD_ShowString_trans(10, INFO1_y, 240, INFO1_s, INFO1_s, "data len: ");
    LCD_ShowNum(10+10*6, INFO1_y, data_length, num_len(data_length), INFO1_s);
    LCD_ShowString_trans(10, INFO2_y, 240, INFO2_s, INFO2_s, "Hamm len: ");
    LCD_ShowNum(10+10*6, INFO2_y, true_length, num_len(true_length), INFO2_s);
  }
  switch (mode)
  {
  case Encode_Mode:
    if(refresh_area & INPUT_REFRESH){ // repaint input area
      refresh_area ^= INPUT_REFRESH;
      LCD_Fill(0, INPUT_data_y, 240, INPUT_data_y+INPUT_data_s, BACK_COLOR);
      char *ss = my_data2string(data, data_ptr);
      show_data_string(INPUT_data_y, INPUT_data_s, ss);
      free(ss);
    }
    if(refresh_area & OUTPUT_REFRESH){
      refresh_area ^= OUTPUT_REFRESH;
      LCD_Fill(0, OUTPUT_data_y, 240, OUTPUT_data_y+OUTPUT_data_s, BACK_COLOR);
      if (data_ptr == data_length){ // show the encoded result
        uint32_t *enc = encode_data(data);
        char *ss = my_data2string(enc, true_length);
        show_data_string(OUTPUT_data_y, OUTPUT_data_s, ss);
        LCD_ShowString_trans(10, ADDI_msg_y, 220, ADDI_msg_s, ADDI_msg_s, "No error");
        free(ss);
        free(enc);
      }
    }
    if(refresh_area & ADDI_REFRESH){
      refresh_area ^= ADDI_REFRESH;
      LCD_Fill(0, ADDI_msg_y, 240, ADDI_msg_y+ADDI_msg_s, BACK_COLOR);
      LCD_Fill(0, ADDI_err_y-2, 240, ADDI_err_y+2+ADDI_err_s, BACK_COLOR);
      LCD_Fill(0, ADDI_cor_y-2, 240, ADDI_cor_y+2+ADDI_cor_s, BACK_COLOR);
      if (data_ptr == data_length){
        LCD_ShowString_trans(10, ADDI_msg_y, 220, ADDI_msg_s, ADDI_msg_s, "No error");
      }
    }
    break;
  case Decode_Mode:
    if(refresh_area & INPUT_REFRESH){
      refresh_area ^= INPUT_REFRESH;
      LCD_Fill(0, OUTPUT_data_y, 240, OUTPUT_data_y+OUTPUT_data_s, BACK_COLOR);

      char *ss = my_data2string(data, data_ptr);
      show_data_string(OUTPUT_data_y, OUTPUT_data_s, ss);
    }
    if(refresh_area & OUTPUT_REFRESH){
      refresh_area ^= OUTPUT_REFRESH;

      LCD_Fill(0, INPUT_data_y, 240, INPUT_data_y+INPUT_data_s, BACK_COLOR);
      LCD_Fill(0, ADDI_msg_y, 240, ADDI_msg_y+ADDI_msg_s, BACK_COLOR);
      LCD_Fill(0, ADDI_err_y-2, 240, ADDI_err_y+2+ADDI_err_s, BACK_COLOR);
      LCD_Fill(0, ADDI_cor_y-2, 240, ADDI_cor_y+2+ADDI_cor_s, BACK_COLOR);

      if (data_ptr == true_length){ // show the decoded result
        enum Decode_Result dr;
        size_t pos, decl, U;
        uint32_t *dec = decode_data(data, &dr, &pos, &decl, &U);
        char *ss = my_data2string(dec, decl);
        
        show_warning_msg(U ? "Intermediate: U=1" : "Intermediate: U=0"); // All intermediate variables not shown only remains U...
        LCD_ShowString_trans(10 + 9 * WARN_s, WARN_y, 220, WARN_s, WARN_s, "P=");
        char *sss = my_2binary(pos);
        LCD_ShowString_trans(10 + 10 * WARN_s, WARN_y, 220, WARN_s, WARN_s, sss);
        free(sss);
        if(dr == TwoBit){
          LCD_ShowString_trans(10, ADDI_msg_y, 220, ADDI_msg_s, ADDI_msg_s, "2 or more errors");
        } else if(dr == OneBit){
          LCD_ShowString_trans(10, ADDI_msg_y, 220, ADDI_msg_s, ADDI_msg_s, "1");
          LCD_ShowString_trans(10+8, ADDI_msg_y+4, 220, 12, 12, " error occurs, P=");
          LCD_ShowNum(10+8+17*6, ADDI_msg_y, pos, num_len(pos), ADDI_msg_s);
          LCD_Fill(8, ADDI_err_y-2, 224, ADDI_err_y+ADDI_err_s+2, RED);
          LCD_Fill(8, ADDI_cor_y-2, 224, ADDI_cor_y+ADDI_cor_s+2, GREEN);

          BACK_COLOR = RED;
          show_data_string(ADDI_err_y, ADDI_err_s, ss); // show error data
          free(ss);
          free(dec);

          uint8_t x = get_databit(data, pos);
          set_databit(data, pos, x^1); // get correct hamming code
          dec = decode_data(data, &dr, &pos, &decl, &U);
          ss = my_data2string(dec, decl);
          BACK_COLOR = GREEN;
          show_data_string(ADDI_cor_y, ADDI_cor_s, ss); // show correct data

          BACK_COLOR = WHITE;
          show_data_string(INPUT_data_y, INPUT_data_s, ss);

          if(dr != CORRECT){
            show_warning_msg("cannot correct 1 bit error");
          }
        } else {
          show_data_string(INPUT_data_y, INPUT_data_s, ss);
          LCD_Fill(0, ADDI_msg_y, 240, ADDI_msg_y+ADDI_msg_s, BACK_COLOR);
          LCD_ShowString_trans(10, ADDI_msg_y, 220, ADDI_msg_s, ADDI_msg_s, "No error");
        }
        free(ss);
        free(dec);
      }
    }
    break;
  case Set_Mode: // just clear all the things, only refresh length
    LCD_Fill(0, OUTPUT_msg2_y, 240, OUTPUT_msg2_y+OUTPUT_msg2_s, BACK_COLOR);
    size_t numl = num_len(true_length-data_length);
    LCD_ShowString_trans(10, OUTPUT_msg2_y+4, 220, 12, 12, "with ");
    LCD_ShowNum(10+ 5*6, OUTPUT_msg2_y, true_length-data_length, numl, OUTPUT_msg2_s);
    LCD_ShowString_trans(10 + 5*6 + numl*8, OUTPUT_msg2_y+4, 220, 12, 12, " parity bits.");

    if(refresh_area & ADDI_REFRESH){
      refresh_area ^= ADDI_REFRESH;
      LCD_Fill(0, ADDI_msg_y, 240, ADDI_msg_y+ADDI_msg_s, BACK_COLOR);
      LCD_Fill(0, ADDI_err_y-2, 240, ADDI_err_y+2+ADDI_err_s, BACK_COLOR);
      LCD_Fill(0, ADDI_cor_y-2, 240, ADDI_cor_y+2+ADDI_cor_s, BACK_COLOR);
    }
    if(refresh_area & INPUT_REFRESH){
      refresh_area ^= INPUT_REFRESH;
      LCD_Fill(0, INPUT_data_y, 240, INPUT_data_y+INPUT_data_s, BACK_COLOR);
    }
    if(refresh_area & OUTPUT_REFRESH){
      refresh_area ^= OUTPUT_REFRESH;
      LCD_Fill(0, OUTPUT_data_y, 240, OUTPUT_data_y+OUTPUT_data_s, BACK_COLOR);
    }
    break;
  
  default:
    break;
  }
  if(refresh_area & WARN_REFRESH){
    refresh_area ^= WARN_REFRESH;
    LCD_Fill(0, WARN_y, 240, WARN_y+WARN_s, BACK_COLOR);
  }
}

void show_cover(){
  LCD_Color_Fill(0, 0, 239, 319, gImage_dmx_100);
  LCD_ShowString_trans(18, 50, 220, 24, 24, "Hamming Code Tool");
  LCD_ShowString_trans(135, 84, 220, 12, 12, "by XJC 12112012");

  LCD_DrawLine(10, 105, 220, 105);

  LCD_ShowString_trans(10, 115, 220, 12, 12, "This program is capable of");
  LCD_ShowString_trans(10, 115+17, 220, 12, 12, "  performing 2bit detection using");
  LCD_ShowString_trans(10, 115+17*2, 220, 12, 12, "  an extra parity bit. Also can");
  LCD_ShowString_trans(10, 115+17*3, 220, 12, 12, "  adjust input length in Set Mode.");
  LCD_ShowString_trans(10, 115+17*4+20, 220, 12, 12, "Use WK_UP to enter and switch mode");
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
