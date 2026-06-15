#ifndef __WONDER_MV_H_
#define __WONDER_MV_H_

#include "stdint.h"
#include "stdbool.h"

#define WONDERMV_ADDR  0x32
#define COLOR_REG  	   0x00
//#define COLOR_REG  	   0x01
#define FACE_REG  	   0x10
#define TAG_REG  	   0x20
#define OBJECT_REG     0x30

#define COLOR_RED_REG     0x10
#define COLOR_GREEN_REG   0x19
#define COLOR_BLUE_REG    0x22

#define NUMBER_1_REG      0x40
#define NUMBER_2_REG      0x49
#define NUMBER_3_REG      0x52
#define NUMBER_4_REG      0x5B
#define NUMBER_5_REG      0x64

#define RECOGNITION_RESULT_SIZE        9U
#define NUMBER_RESULT_COUNT            5U
#define COLOR_NUMBER_RESULT_SIZE       (RECOGNITION_RESULT_SIZE * (1U + NUMBER_RESULT_COUNT))

typedef struct
{
	uint16_t w;
	uint16_t h;
	uint16_t x;
	uint16_t y;
}PositionObjectTypeDef;

typedef struct
{
	uint8_t id;
	PositionObjectTypeDef position;
}RecognitionHanleTypeDef;

typedef struct WonderMVHandle WonderMVHandleTypeDef;
struct WonderMVHandle
{
	uint16_t dev_addr;
	
	uint8_t transmit_status;
	uint8_t receive_status;
	
	uint8_t results[RECOGNITION_RESULT_SIZE];
	uint8_t color_number_results[COLOR_NUMBER_RESULT_SIZE];
	
	uint8_t (*write_data)(WonderMVHandleTypeDef* self, uint8_t* pdata, uint16_t size);
	uint8_t (*read_data)(WonderMVHandleTypeDef* self, uint8_t* pdata, uint16_t size);
	
};

/**
 * @brief wonder mv接口初始化
 */
void wonder_mv_init(void);

/**
 * @brief 颜色识别
 * 
 * @param  color 指向RecognitionHanleTypeDef类型的指针
 * @return true 
 * @return false 
 */
bool wonder_mv_color_recognition(RecognitionHanleTypeDef* color);
bool wonder_mv_color_number_recognition(RecognitionHanleTypeDef* color, RecognitionHanleTypeDef* numbers);
bool wonder_mv_read_recognition(uint8_t reg, RecognitionHanleTypeDef* result);
bool wonder_mv_color_recognition_by_id(uint8_t color_id, RecognitionHanleTypeDef* color);
bool wonder_mv_number_recognition_by_id(uint8_t number_id, RecognitionHanleTypeDef* number);

/**
 * @brief 人脸识别
 * 
 * @param  face
 * @return true 
 * @return false 
 */
bool wonder_mv_face_detection(RecognitionHanleTypeDef* face);

/**
 * @brief 标签识别
 * 
 * @param  tag
 * @return true 
 * @return false 
 */
bool wonder_mv_tag_detection(RecognitionHanleTypeDef* tag);

/**
 * @brief 物体识别
 * 
 * @param  obj
 * @return true 
 * @return false 
 */
bool wonder_mv_object_detection(RecognitionHanleTypeDef* obj);
#endif
