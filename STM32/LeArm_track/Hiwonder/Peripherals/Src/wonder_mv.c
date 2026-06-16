#include "wonder_mv.h"
#include "string.h"
#include "global.h"
#include "i2c.h"

/*
 * [0]:id
 * [1]:x(low 8bit) 
 * [2]:x(high 8bit) 
 * [3]:y(low 8bit) 
 * [4]:y(high 8bit) 
 * [5]:w(low 8bit) 
 * [6]:w(high 8bit) 
 * [7]:h(low 8bit) 
 * [8]:h(high 8bit)
 */
 
WonderMVHandleTypeDef wonder_mv;

#define WONDERMV_I2C_READY_TIMEOUT_MS  20U
#define WONDERMV_I2C_RETRY_DELAY_MS     2U
	
static uint8_t write_data(WonderMVHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{
	if(!i2c1_wait_ready(WONDERMV_I2C_READY_TIMEOUT_MS))
	{
		i2c1_recover();
	}

	self->transmit_status = (uint8_t)HAL_I2C_Master_Transmit(&hi2c1, self->dev_addr << 1, pdata, size, 0xfff);
	return self->transmit_status;
}

static uint8_t read_data(WonderMVHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{
	if(!i2c1_wait_ready(WONDERMV_I2C_READY_TIMEOUT_MS))
	{
		i2c1_recover();
	}

	/* 使用HAL_I2C_Master_Receive_DMA这个函数需要把i2c的事件中断勾上 */
	self->receive_status = (uint8_t)HAL_I2C_Master_Receive_DMA(&hi2c1, self->dev_addr << 1, pdata, size);
	return self->receive_status;	
}


static bool wait_receive_complete(void)
{
	if(!i2c1_wait_ready(WONDERMV_I2C_READY_TIMEOUT_MS))
	{
		i2c1_recover();
		return false;
	}

	return true;
}

static bool write_to_device(WonderMVHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
{
	uint8_t trans_data[size + 1];
	
	trans_data[0] = reg;
	
	for (uint16_t i = 0; i < size; i++)
	{
		trans_data[1 + i] = pdata[i];
	}
	
	if(write_data(self, trans_data, sizeof(trans_data)) != 0)
	{
		HAL_Delay(WONDERMV_I2C_RETRY_DELAY_MS);
		i2c1_recover();
		return write_data(self, trans_data, sizeof(trans_data)) == 0;
	}
	
	return true;
}

static bool receive_from_device(WonderMVHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
{
	uint8_t set_reg= reg;	
	
	if(write_data(self, &set_reg, 1) != 0)
	{
		HAL_Delay(WONDERMV_I2C_RETRY_DELAY_MS);
		i2c1_recover();
		if(write_data(self, &set_reg, 1) != 0)
		{
			return false;
		}
	}
	
	if(read_data(self, pdata, size) != 0)
	{
		HAL_Delay(WONDERMV_I2C_RETRY_DELAY_MS);
		i2c1_recover();
		if(write_data(self, &set_reg, 1) != 0)
		{
			return false;
		}
		return read_data(self, pdata, size) == 0;
	}
	
	return true;
}

void wonder_mv_init()
{
	memset(&wonder_mv, 0, sizeof(WonderMVHandleTypeDef));
	wonder_mv.dev_addr = WONDERMV_ADDR;
	wonder_mv.read_data = read_data;
	wonder_mv.write_data = write_data;
}

static void decode_recognition_result_from_buffer(const uint8_t* data, RecognitionHanleTypeDef* result)
{
	result->id = data[0];
	result->position.x = BYTE_TO_HW(data[2], data[1]);
	result->position.y = BYTE_TO_HW(data[4], data[3]);
	result->position.w = BYTE_TO_HW(data[6], data[5]);
	result->position.h = BYTE_TO_HW(data[8], data[7]);
}

static void decode_recognition_result(RecognitionHanleTypeDef* result)
{
	decode_recognition_result_from_buffer(wonder_mv.results, result);
}

bool wonder_mv_read_recognition(uint8_t reg, RecognitionHanleTypeDef* result)
{
	if(receive_from_device(&wonder_mv, reg, wonder_mv.results, sizeof(wonder_mv.results)))
	{
		if(!wait_receive_complete())
		{
			return false;
		}
		decode_recognition_result(result);
		return true;
	}

	return false;
}

static uint8_t color_id_to_reg(uint8_t color_id)
{
	switch(color_id)
	{
		case 1:
			return COLOR_RED_REG;
		case 2:
			return COLOR_GREEN_REG;
		case 3:
			return COLOR_BLUE_REG;
		default:
			return 0;
	}
}

static uint8_t number_id_to_reg(uint8_t number_id)
{
	switch(number_id)
	{
		case 1:
			return NUMBER_1_REG;
		case 2:
			return NUMBER_2_REG;
		case 3:
			return NUMBER_3_REG;
		case 4:
			return NUMBER_4_REG;
		case 5:
			return NUMBER_5_REG;
		default:
			return 0;
	}
}

bool wonder_mv_color_recognition(RecognitionHanleTypeDef* color)
{
	return wonder_mv_read_recognition(COLOR_REG, color);
}

bool wonder_mv_color_number_recognition(RecognitionHanleTypeDef* colors, RecognitionHanleTypeDef* numbers)
{
	if(receive_from_device(&wonder_mv, COLOR_REG, wonder_mv.color_number_results, sizeof(wonder_mv.color_number_results)))
	{
		if(!wait_receive_complete())
		{
			return false;
		}

		for(uint8_t i = 0; i < COLOR_RESULT_COUNT; i++)
		{
			decode_recognition_result_from_buffer(
				&wonder_mv.color_number_results[RECOGNITION_RESULT_SIZE * i],
				&colors[i]
			);
		}

		for(uint8_t i = 0; i < NUMBER_RESULT_COUNT; i++)
		{
			decode_recognition_result_from_buffer(
				&wonder_mv.color_number_results[RECOGNITION_RESULT_SIZE * (COLOR_RESULT_COUNT + i)],
				&numbers[i]
			);
		}
		return true;
	}

	return false;
}

bool wonder_mv_color_recognition_by_id(uint8_t color_id, RecognitionHanleTypeDef* color)
{
	uint8_t reg = color_id_to_reg(color_id);

	if(reg == 0 || !wonder_mv_read_recognition(reg, color))
	{
		return false;
	}

	return color->id == color_id;
}

bool wonder_mv_number_recognition_by_id(uint8_t number_id, RecognitionHanleTypeDef* number)
{
	uint8_t reg = number_id_to_reg(number_id);

	if(reg == 0 || !wonder_mv_read_recognition(reg, number))
	{
		return false;
	}

	return number->id == number_id;
}


bool wonder_mv_face_detection(RecognitionHanleTypeDef* face)
{
	return wonder_mv_read_recognition(FACE_REG, face);
}

bool wonder_mv_tag_detection(RecognitionHanleTypeDef* tag)
{
	return wonder_mv_read_recognition(TAG_REG, tag);
}

bool wonder_mv_object_detection(RecognitionHanleTypeDef* obj)
{
	return wonder_mv_read_recognition(OBJECT_REG, obj);
}
