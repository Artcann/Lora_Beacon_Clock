#include "rfm95.h"
#include "lib/ideetron/Encrypt_V31.h"

#include <assert.h>
#include <string.h>

#define RFM9x_VER 0x12

/**
 * Registers addresses.
 */
typedef enum
{
	RFM95_REGISTER_FIFO_ACCESS = 0x00,
	RFM95_REGISTER_OP_MODE = 0x01,
	RFM95_REGISTER_FR_MSB = 0x06,
	RFM95_REGISTER_FR_MID = 0x07,
	RFM95_REGISTER_FR_LSB = 0x08,
	RFM95_REGISTER_PA_CONFIG = 0x09,
	RFM95_REGISTER_LNA = 0x0C,
	RFM95_REGISTER_FIFO_ADDR_PTR = 0x0D,
	RFM95_REGISTER_FIFO_TX_BASE_ADDR = 0x0E,
	RFM95_REGISTER_FIFO_RX_BASE_ADDR = 0x0F,
	RFM95_REGISTER_FIFO_RX_CURRENT_ADDR = 0x10,
	RFM95_REGISTER_IRQ_FLAGS = 0x12,
	RFM95_REGISTER_FIFO_RX_BYTES_NB = 0x13,
	RFM95_REGISTER_PACKET_SNR = 0x19,
	RFM95_REGISTER_MODEM_CONFIG_1 = 0x1D,
	RFM95_REGISTER_MODEM_CONFIG_2 = 0x1E,
	RFM95_REGISTER_SYMB_TIMEOUT_LSB = 0x1F,
	RFM95_REGISTER_PREAMBLE_MSB = 0x20,
	RFM95_REGISTER_PREAMBLE_LSB = 0x21,
	RFM95_REGISTER_PAYLOAD_LENGTH = 0x22,
	RFM95_REGISTER_MAX_PAYLOAD_LENGTH = 0x23,
	RFM95_REGISTER_MODEM_CONFIG_3 = 0x26,
	RFM95_REGISTER_INVERT_IQ_1 = 0x33,
	RFM95_REGISTER_SYNC_WORD = 0x39,
	RFM95_REGISTER_INVERT_IQ_2 = 0x3B,
	RFM95_REGISTER_DIO_MAPPING_1 = 0x40,
	RFM95_REGISTER_VERSION = 0x42,
	RFM95_REGISTER_PA_DAC = 0x4D
} rfm95_register_t;

typedef struct
{
	union {
		struct {
			uint8_t output_power : 4;
			uint8_t max_power : 3;
			uint8_t pa_select : 1;
		};
		uint8_t buffer;
	};
} rfm95_register_pa_config_t;

#define RFM95_REGISTER_OP_MODE_SLEEP                            0x00
#define RFM95_REGISTER_OP_MODE_LORA_SLEEP                       0x80
#define RFM95_REGISTER_OP_MODE_LORA_STANDBY                     0x81
#define RFM95_REGISTER_OP_MODE_LORA_TX                          0x83
#define RFM95_REGISTER_OP_MODE_RX_CONTINUOUS					0x85
#define RFM95_REGISTER_OP_MODE_LORA_RX_SINGLE                   0x86

#define RFM95_REGISTER_PA_DAC_LOW_POWER                         0x84
#define RFM95_REGISTER_PA_DAC_HIGH_POWER                        0x87

#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE             0x40
#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE             0x00

#define RFM95_REGISTER_INVERT_IQ_1_TX                    		0x27
#define RFM95_REGISTER_INVERT_IQ_2_TX							0x1d

#define RFM95_REGISTER_INVERT_IQ_1_RX                    		0x67
#define RFM95_REGISTER_INVERT_IQ_2_RX							0x19

static bool read_register(rfm95_handle_t *handle, rfm95_register_t reg, uint8_t *buffer, size_t length)
{
	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);

	uint8_t transmit_buffer = (uint8_t)reg & 0x7fu;

	if (HAL_SPI_Transmit(handle->spi_handle, &transmit_buffer, 1, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	if (HAL_SPI_Receive(handle->spi_handle, buffer, length, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

	return true;
}

static bool write_register(rfm95_handle_t *handle, rfm95_register_t reg, uint8_t value)
{
	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);

	uint8_t transmit_buffer[2] = {((uint8_t)reg | 0x80u), value};

	if (HAL_SPI_Transmit(handle->spi_handle, transmit_buffer, 2, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

	return true;
}

static void reset(rfm95_handle_t *handle)
{
	HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_RESET);
	HAL_Delay(1); // 0.1ms would theoretically be enough
	HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_SET);
	HAL_Delay(5);
}

static bool configure_frequency(rfm95_handle_t *handle, uint32_t frequency)
{
	// FQ = (FRF * 32 Mhz) / (2 ^ 19)
	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

	if (!write_register(handle, RFM95_REGISTER_FR_MSB, (uint8_t)(frf >> 16))) return false;
	if (!write_register(handle, RFM95_REGISTER_FR_MID, (uint8_t)(frf >> 8))) return false;
	if (!write_register(handle, RFM95_REGISTER_FR_LSB, (uint8_t)(frf >> 0))) return false;

	return true;
}

bool rfm95_set_power(rfm95_handle_t *handle, int8_t power)
{
	assert((power >= 2 && power <= 17) || power == 20);

	rfm95_register_pa_config_t pa_config = {0};
	uint8_t pa_dac_config = 0;

	if (power >= 2 && power <= 17) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = (power - 2);
		pa_dac_config = RFM95_REGISTER_PA_DAC_LOW_POWER;

	} else if (power == 20) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = 15;
		pa_dac_config = RFM95_REGISTER_PA_DAC_HIGH_POWER;
	}

	if (!write_register(handle, RFM95_REGISTER_PA_CONFIG, pa_config.buffer)) return false;
	if (!write_register(handle, RFM95_REGISTER_PA_DAC, pa_dac_config)) return false;

	return true;
}

bool rfm95_init(rfm95_handle_t *handle, UART_HandleTypeDef *uart_handle)
{

	assert(handle->spi_handle->Init.Mode == SPI_MODE_MASTER);
	assert(handle->spi_handle->Init.Direction == SPI_DIRECTION_2LINES);
	assert(handle->spi_handle->Init.DataSize == SPI_DATASIZE_8BIT);
	assert(handle->spi_handle->Init.CLKPolarity == SPI_POLARITY_LOW);
	assert(handle->spi_handle->Init.CLKPhase == SPI_PHASE_1EDGE);

	reset(handle);




	// Check for correct version.
	uint8_t version;
	if (!read_register(handle, RFM95_REGISTER_VERSION, &version, 1)) return false;
	if (version != RFM9x_VER) return false;


	// Module must be placed in sleep mode before switching to lora.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_SLEEP)) return false;
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	// Default interrupt configuration, must be done to prevent DIO5 clock interrupts at 1Mhz
	if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;

	if (handle->on_after_interrupts_configured != NULL) {
		handle->on_after_interrupts_configured();
	}

	// Set module power to 17dbm.
	if (!rfm95_set_power(handle, 17)) return false;

	// Set LNA to the highest gain with 150% boost.
	if (!write_register(handle, RFM95_REGISTER_LNA, 0x23)) return false;

	// Preamble set to 8 + 4.25 = 12.25 symbols.
	if (!write_register(handle, RFM95_REGISTER_PREAMBLE_MSB, 0x00)) return false;
	if (!write_register(handle, RFM95_REGISTER_PREAMBLE_LSB, 0x08)) return false;

	// Set TTN sync word 0x34.
	if (!write_register(handle, RFM95_REGISTER_SYNC_WORD, 0x34)) return false;

	// Set up TX and RX FIFO base addresses.
	if (!write_register(handle, RFM95_REGISTER_FIFO_TX_BASE_ADDR, 0x80)) return false;
	if (!write_register(handle, RFM95_REGISTER_FIFO_RX_BASE_ADDR, 0x00)) return false;

	// Maximum payload length of the RFM95 is 64.
	if (!write_register(handle, RFM95_REGISTER_MAX_PAYLOAD_LENGTH, 64)) return false;

	if(!configure_frequency(handle, 868100000)) return false;

	// Let module sleep after initialisation.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	return true;
}


bool receive_package(rfm95_handle_t *handle, uint8_t *payload_buf, size_t *payload_len, int8_t *snr, UART_HandleTypeDef *uart_handle) {
	*payload_len = 0;


	//Stand-by mode previous to Continuous Mode
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return false;

	// Configure modem (125kHz, 4/6 error coding rate, SF7, single packet, CRC enable, AGC auto on)
	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x72)) return false;
	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0xC2)) return false;
	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, 0x04)) return false;


	// Set maximum symbol timeout.
	//if (!write_register(handle, RFM95_REGISTER_SYMB_TIMEOUT_LSB, 16)) return false;

	// Set IQ registers according to AN1200.24.
	if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_1, RFM95_REGISTER_INVERT_IQ_1_TX)) return false;
	if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_2, RFM95_REGISTER_INVERT_IQ_2_TX)) return false;

	// receive_at_scheduled_time(handle, rx1_target);


	if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;
	if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xff)) return false;
	handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;
	handle->interrupt_times[RFM95_INTERRUPT_DIO1] = 0;
	handle->interrupt_times[RFM95_INTERRUPT_DIO5] = 0;



	// Clear flags
	uint8_t irqFlags;

	// Continuous Mode
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_RX_SINGLE)) return false;


	while ((irqFlags != 0x50)) { // wait for RxDone
	  HAL_Delay(10);
	  read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &irqFlags, 1); // read after the delay, not before

	  HAL_UART_Transmit(uart_handle, irqFlags, sizeof(irqFlags), 10);

	  if (irqFlags & 0xA0) return false; // cancel on PayloadCrcError and RxTimeout
	}

	HAL_UART_Transmit(uart_handle, irqFlags, sizeof(irqFlags), 10);

	 // Read received payload length.
	uint8_t payload_len_internal;
	if (!read_register(handle, RFM95_REGISTER_FIFO_RX_BYTES_NB, &payload_len_internal, 1)) return false;

	uint8_t payload[payload_len_internal];


	 // Read received payload itself.
	if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, 0)) return false;
	if (!read_register(handle, RFM95_REGISTER_FIFO_ACCESS, payload, payload_len_internal)) return false;

	HAL_UART_Transmit(uart_handle, payload, payload_len_internal, 10);
	HAL_Delay(100);


	// Return modem to sleep.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;


	return true;
}

void rfm95_on_interrupt(rfm95_handle_t *handle, rfm95_interrupt_t interrupt)
{
	handle->interrupt_times[interrupt] = handle->get_precision_tick();
}
