#include "wonder_mv.h"
#include "string.h"
#include "global.h"
#include "i2c.h"

/*
 * I2C register result layout:
 * [0]: id
 * [1]: x low 8 bit
 * [2]: x high 8 bit
 * [3]: y low 8 bit
 * [4]: y high 8 bit
 * [5]: w low 8 bit
 * [6]: w high 8 bit
 * [7]: h low 8 bit
 * [8]: h high 8 bit
 */

WonderMVHandleTypeDef wonder_mv;

static uint8_t write_data(WonderMVHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{
	self->transmit_status = (uint8_t)HAL_I2C_Master_Transmit(&hi2c1, self->dev_addr << 1, pdata, size, 0xfff);
	return self->transmit_status;
}

static uint8_t read_data(WonderMVHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{
	self->receive_status = (uint8_t)HAL_I2C_Master_Receive(&hi2c1, self->dev_addr << 1, pdata, size, 0xfff);
	return self->receive_status;
}

static bool write_to_device(WonderMVHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
{
	uint8_t trans_data[10];

	if (size > (sizeof(trans_data) - 1))
	{
		return false;
	}

	trans_data[0] = reg;

	for (uint16_t i = 0; i < size; i++)
	{
		trans_data[1 + i] = pdata[i];
	}

	if(write_data(self, trans_data, size + 1) != 0)
	{
		return false;
	}

	return true;
}

static bool receive_from_device(WonderMVHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
{
	uint8_t set_reg = reg;

	if(write_data(self, &set_reg, 1) != 0)
	{
		return false;
	}

	if(read_data(self, pdata, size) != 0)
	{
		return false;
	}

	return true;
}

static void recognition_from_results(RecognitionHanleTypeDef* out, const uint8_t* results)
{
	out->id = results[0];
	out->position.x = BYTE_TO_HW(results[2], results[1]);
	out->position.y = BYTE_TO_HW(results[4], results[3]);
	out->position.w = BYTE_TO_HW(results[6], results[5]);
	out->position.h = BYTE_TO_HW(results[8], results[7]);
}

void wonder_mv_init(void)
{
	memset(&wonder_mv, 0, sizeof(WonderMVHandleTypeDef));
	wonder_mv.dev_addr = WONDERMV_ADDR;
	wonder_mv.read_data = read_data;
	wonder_mv.write_data = write_data;
}

bool wonder_mv_color_recognition(RecognitionHanleTypeDef* color)
{
	if(color == NULL)
	{
		return false;
	}

	if(receive_from_device(&wonder_mv, COLOR_REG, wonder_mv.results, sizeof(wonder_mv.results)))
	{
		recognition_from_results(color, wonder_mv.results);
		return true;
	}

	memset(color, 0, sizeof(RecognitionHanleTypeDef));
	return false;
}

bool wonder_mv_face_detection(RecognitionHanleTypeDef* face)
{
	if(face == NULL)
	{
		return false;
	}

	if(receive_from_device(&wonder_mv, FACE_REG, wonder_mv.results, sizeof(wonder_mv.results)))
	{
		recognition_from_results(face, wonder_mv.results);
		return true;
	}

	memset(face, 0, sizeof(RecognitionHanleTypeDef));
	return false;
}

bool wonder_mv_tag_detection(RecognitionHanleTypeDef* tag)
{
	if(tag == NULL)
	{
		return false;
	}

	if(receive_from_device(&wonder_mv, TAG_REG, wonder_mv.results, sizeof(wonder_mv.results)))
	{
		recognition_from_results(tag, wonder_mv.results);
		return true;
	}

	memset(tag, 0, sizeof(RecognitionHanleTypeDef));
	return false;
}

bool wonder_mv_object_detection(RecognitionHanleTypeDef* obj)
{
	if(obj == NULL)
	{
		return false;
	}

	if(receive_from_device(&wonder_mv, OBJECT_REG, wonder_mv.results, sizeof(wonder_mv.results)))
	{
		recognition_from_results(obj, wonder_mv.results);
		return true;
	}

	memset(obj, 0, sizeof(RecognitionHanleTypeDef));
	return false;
}
