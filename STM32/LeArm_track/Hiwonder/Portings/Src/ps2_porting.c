#include "ps2_porting.h"
#include "usart.h"

#ifdef MECANUM_CHASSIS
#include "mecanum_chassis.h"
#endif

PS2HandleTypeDef ps2;

#ifdef MECANUM_CHASSIS
static int8_t rot;
static uint8_t speed;
static uint16_t angle;
#endif

static void packet_uart_error_callblack(UART_HandleTypeDef *huart);
static void packet_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length);

static void receive_data(uint8_t* pdata, uint16_t size)
{
	HAL_UART_AbortReceive(&huart3);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, pdata, size);
}

static void packet_start_receive(void)
{
	HAL_UART_RegisterCallback(&huart3, HAL_UART_ERROR_CB_ID, packet_uart_error_callblack);
	HAL_UART_RegisterRxEventCallback(&huart3, packet_dma_receive_event_callback);
	ps2.receive_data(ps2.rx_dma_buf, sizeof(ps2.rx_dma_buf));
}

static void packet_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length)
{
	lwrb_write(&ps2.rb, ps2.rx_dma_buf, length);
	ps2.receive_data(ps2.rx_dma_buf, sizeof(ps2.rx_dma_buf));
}

static void packet_uart_error_callblack(UART_HandleTypeDef *huart)
{
	packet_start_receive();
}

static void ps2_stop_chassis(void)
{
#ifdef MECANUM_CHASSIS
	angle = 0;
	speed = 0;
	rot = 0;
	mecanum_chassis_run(angle, speed, rot, 0);
#endif
}

static void ps2_update_mode(PS2HandleTypeDef* self)
{
	uint8_t last_mode = self->mode;

	if(self->keyvalue.bit_mode == 1 && self->last_keyvalue.bit_mode == 0)
	{
		self->mode_button_status = !self->mode_button_status;
		self->mode = self->mode_button_status == 0 ? PS2_AUTO_MODE : PS2_CHASSIS_MODE;

		if(last_mode == PS2_CHASSIS_MODE && self->mode == PS2_AUTO_MODE)
		{
			ps2_stop_chassis();
		}
	}
}

static void ps2_decode_direction(PS2HandleTypeDef* self)
{
	switch(self->packet.buffer[4])
	{
		case 0x00:
			self->keyvalue.bit_up = 1;
			break;

		case 0x01:
			self->keyvalue.bit_up = 1;
			self->keyvalue.bit_right = 1;
			break;

		case 0x02:
			self->keyvalue.bit_right = 1;
			break;

		case 0x03:
			self->keyvalue.bit_down = 1;
			self->keyvalue.bit_right = 1;
			break;

		case 0x04:
			self->keyvalue.bit_down = 1;
			break;

		case 0x05:
			self->keyvalue.bit_down = 1;
			self->keyvalue.bit_left = 1;
			break;

		case 0x06:
			self->keyvalue.bit_left = 1;
			break;

		case 0x07:
			self->keyvalue.bit_up = 1;
			self->keyvalue.bit_left = 1;
			break;

		default:
			break;
	}
}

static void ps2_run_chassis(PS2HandleTypeDef* self)
{
	if(self->mode != PS2_CHASSIS_MODE)
	{
		return;
	}

#ifdef MECANUM_CHASSIS
	if(self->keyvalue.left_joystick_x == 127 && self->keyvalue.left_joystick_y == 127)
	{
		angle = 0;
		speed = 0;
	}
	else if(self->keyvalue.left_joystick_x == 127 && self->keyvalue.left_joystick_y < 127)
	{
		angle = 180;
		speed = 80;
	}
	else if(self->keyvalue.left_joystick_x == 127 && self->keyvalue.left_joystick_y > 127)
	{
		angle = 0;
		speed = 80;
	}
	else if(self->keyvalue.left_joystick_x < 127 && self->keyvalue.left_joystick_y == 127)
	{
		angle = 270;
		speed = 80;
	}
	else if(self->keyvalue.left_joystick_x > 127 && self->keyvalue.left_joystick_y == 127)
	{
		angle = 90;
		speed = 80;
	}
	else if(self->keyvalue.left_joystick_x < 127 && self->keyvalue.left_joystick_y < 127)
	{
		angle = 225;
		speed = 80;
	}
	else if(self->keyvalue.left_joystick_x < 127 && self->keyvalue.left_joystick_y > 127)
	{
		angle = 315;
		speed = 80;
	}
	else if(self->keyvalue.left_joystick_x > 127 && self->keyvalue.left_joystick_y > 127)
	{
		angle = 45;
		speed = 80;
	}
	else if(self->keyvalue.left_joystick_x > 127 && self->keyvalue.left_joystick_y < 127)
	{
		angle = 135;
		speed = 80;
	}

	if(self->keyvalue.right_joystick_x == 127)
	{
		rot = 0;
	}
	else if(self->keyvalue.right_joystick_x < 127)
	{
		rot = -80;
	}
	else if(self->keyvalue.right_joystick_x > 127)
	{
		rot = 80;
	}

	mecanum_chassis_run(angle, speed, rot, 0);
#endif
}

void ps2_init(void)
{
	memset(&ps2, 0, sizeof(PS2HandleTypeDef));
	ps2.mode = PS2_AUTO_MODE;
	ps2.mode_button_status = 0;
	ps2.receive_data = receive_data;
	lwrb_init(&ps2.rb, ps2.rx_fifo, sizeof(ps2.rx_fifo));
	packet_start_receive();

#ifdef MECANUM_CHASSIS
	mecanum_chassis_init();
	ps2_stop_chassis();
#endif
}

uint8_t ps2_is_chassis_mode(void)
{
	return ps2.mode == PS2_CHASSIS_MODE;
}

static void unpack(PS2HandleTypeDef* self)
{
	uint8_t i;
	uint32_t readlen = 0;
	uint32_t available = 0;

	static uint8_t buffer_index = 0;
	static uint8_t rec_data[PS2_PACKET_LENGTH] = {0};

	available = lwrb_get_full(&self->rb);
	if(available == 0)
	{
		return;
	}

	available = available > PS2_PACKET_LENGTH ? PS2_PACKET_LENGTH : available;
	if(self->packet_status == PS2_PACKET_HEADER_1)
	{
		self->unpack_status = UNPACK_FINISH;
		readlen = lwrb_read(&self->rb, rec_data, 1);
		self->packet_status = rec_data[0] == PS2_PACKET_HEADER ? PS2_PACKET_HEADER_2 : PS2_PACKET_HEADER_1;
		self->packet.packet_header[0] = rec_data[0];
	}

	if(self->packet_status == PS2_PACKET_HEADER_2)
	{
		readlen = lwrb_read(&self->rb, rec_data, 2);
		for(i = 0; i < readlen; i++)
		{
			switch(self->packet_status)
			{
				case PS2_PACKET_HEADER_2:
					self->packet_status = rec_data[i] == PS2_PACKET_HEADER ? PS2_PACKET_DATA_LENGTH : PS2_PACKET_HEADER_1;
					self->packet.packet_header[1] = rec_data[i];
					break;

				case PS2_PACKET_DATA_LENGTH:
					self->packet_status = rec_data[i] != 0 ? PS2_PACKET_DATA : PS2_PACKET_HEADER_1;
					self->packet.data_len = rec_data[i];
					buffer_index = 0;
					break;

				default:
					self->packet_status = PS2_PACKET_HEADER_1;
					break;
			}
		}
	}

	if(self->packet_status == PS2_PACKET_DATA)
	{
		readlen = lwrb_read(&self->rb, rec_data, self->packet.data_len - 1);
		for(i = 0; i < readlen; i++)
		{
			self->packet.buffer[buffer_index] = rec_data[i];
			buffer_index++;
			if(buffer_index == self->packet.data_len - 2)
			{
				self->packet_status = PS2_PACKET_HEADER_1;
				memset(&self->keyvalue, 0, sizeof(PS2KeyValueObjectTypeDef));
				self->keyvalue.buffer0 = self->packet.buffer[2];
				self->keyvalue.buffer1 = self->packet.buffer[3];
				self->keyvalue.left_joystick_x = self->packet.buffer[5];
				self->keyvalue.left_joystick_y = self->packet.buffer[6];
				self->keyvalue.right_joystick_x = self->packet.buffer[7];
				self->keyvalue.right_joystick_y = self->packet.buffer[8];

				ps2_decode_direction(self);
				ps2_update_mode(self);
				ps2_run_chassis(self);
				self->last_keyvalue = self->keyvalue;
			}
		}
	}
}

void ps2_handler(void)
{
	unpack(&ps2);
}
