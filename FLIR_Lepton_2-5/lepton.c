#include "lepton.h"

/*
 * Start up sequence from uninitialized mode
 * @param : void
 * @return : void
 */
void lepton_startup(){
	PW_DOWN_L_GPIO_Port->ODR |= PW_DOWN_L_Pin;
	HAL_Delay(100);
	RESET_L_GPIO_Port->ODR &= ~RESET_L_Pin;
	HAL_Delay(100);

	RESET_L_GPIO_Port->ODR |= RESET_L_Pin;
	HAL_Delay(100);

	SPI3_CS_GPIO_Port->ODR |= SPI3_CS_Pin;
	HAL_Delay(100);
	SPI3_CS_GPIO_Port->ODR &= ~SPI3_CS_Pin;
	HAL_Delay(5000);
}

/*
 * Reads data from data registers and stores them to given buffer array
 * @param : uint16_t size - number of bytes to read
 * 			uint16_t* buffer - array to contain data
 * @return : void
 */
void lepton_read_data(uint16_t size, uint16_t* buffer){
	uint8_t reg_addr[2] = {0x00, 0x08};
	uint8_t data[size*2];

	HAL_I2C_Master_Transmit(HI2C, I2C_ADDRESS, (uint8_t*)reg_addr, 2, 1000);
	HAL_I2C_Master_Receive(HI2C, I2C_ADDRESS, (uint8_t*)data, size*2, 1000);

	for(int i = 0; i < size; i++){
		buffer[i] = (uint16_t)((data[2*i] << 8) + data[2*i + 1]);
	}
}

uint16_t lepton_run_command(uint16_t command){
	busy_check();

	lepton_write_command(command);

	return busy_check();
}

/*
 * Writes command to command register
 * @param : uint16_t command - command to be writen in command register
 * @return
 */
void lepton_write_command(uint16_t command){
	uint8_t reg_addr[2] = {0x00, 0x04};
	uint8_t data[4] = {reg_addr[0], reg_addr[1], command >> 8, command};

	HAL_I2C_Master_Transmit(HI2C, I2C_ADDRESS, (uint8_t*)data, 4, 1000);
}

/*
 * Writes number of 16-bit data transfers into the data length register
 * @param : uint16_t length - number of 16-bit transfers
 * @return : void
 */
void lepton_write_length(uint16_t length){
	uint8_t reg_addr[2] = {0x00, 0x06};
	uint8_t data[4] = {reg_addr[0], reg_addr[1], length >> 8, length & 0xFF};

	HAL_I2C_Master_Transmit(HI2C, I2C_ADDRESS, (uint8_t*)data, 4, 1000);
}

/*
 * Initializes the control CCI/TWI interface
 * @param : void
 * @return : void
 */
void lepton_command_init(){
	uint8_t reg_addr[2] = {0x00, 0x02};
	uint8_t data[2];
	int retval;
	do{
		retval = HAL_I2C_Master_Transmit(HI2C, I2C_ADDRESS, (uint8_t*)reg_addr, 2, 1000);
		retval = HAL_I2C_Master_Receive(HI2C, I2C_ADDRESS, (uint8_t*)data, 2, 1000);
	}while((data[1] & 0x04) == 0);

}

/*
 * Reads and returns the status register value
 * @param : void
 * @return : uint16_t status - value stored in status register
 */
uint16_t lepton_read_status(){
	uint8_t reg_addr[2] = {0x00, 0x02};
	uint8_t data[2];

	HAL_I2C_Master_Transmit(HI2C, I2C_ADDRESS, (uint8_t*)reg_addr, 2, 1000);
	HAL_I2C_Master_Receive(HI2C, I2C_ADDRESS, (uint8_t*)data, 2, 1000);

	return (uint16_t)((data[0]<<8) + data[1]);
}

void lepton_write_data(uint8_t *data, uint16_t size){
	uint8_t buffer[size*2+2];
	buffer[0] = 0x00;
	buffer[1] = 0x08;

	for(int i = 0; i < size*2; i++){
		buffer[i+2] = data[i];
	}

	HAL_I2C_Master_Transmit(HI2C, I2C_ADDRESS, (uint8_t*)buffer, size*2 + 2, 1000);
}

uint16_t busy_check(){
	uint16_t status = lepton_read_status();
	do{ status = lepton_read_status(); }
	while((status & 0x01) == 1);

	return status;
}

void lepton_write(uint16_t command, uint8_t* data, uint16_t size){
	busy_check();

	lepton_write_data(data, size);
	lepton_write_length(size);
	lepton_write_command(command);

	uint16_t status = busy_check();

	if((status & 0xFF00) != 0){
		HAL_Delay(2000);
	}
}

void lepton_read(uint16_t address, uint16_t* data, uint16_t size){
	busy_check();

	lepton_write_length(size);
	lepton_write_command(address);
	uint16_t status = busy_check();

	if((status & 0xFF00) != 0){
		HAL_Delay(2000);
		return;
	}
	lepton_read_data(size, data);
}

void lepton_read_image_RGB(ili9341_t* _lcd){
	uint8_t buffer[60*244];
	uint16_t image[60*80];
	uint8_t header[4];
	uint16_t IDs[60];

	HAL_SPI_Receive(HSPI, (uint8_t*)header, 4, 1000);

	while((header[0] & 0x0F) == 0x0F){
		HAL_SPI_Receive(HSPI, (uint8_t*)buffer, 240, 1000);
		HAL_SPI_Receive(HSPI, (uint8_t*)header, 4, 1000);
	}

	HAL_SPI_Receive(HSPI, (uint8_t*)&buffer[4], 244*60 - 4, 1000);

	/*
	for(int i = 1; i < 60; i++){
		HAL_SPI_Receive(hspi, (uint8_t*)header, 4, 1000);
		HAL_SPI_Receive(hspi, (uint8_t*)&buffer[i*240], 240, 1000);
		IDs[i] = header[1];
	}*/

	for(int i = 0; i < 60; i++){
		for(int j = 0; j < 80; j++){
			image[i*80+j] = (((uint16_t)buffer[i*244 + j*3 + 4] & 0xF8)<<8)
						  + (((uint16_t)buffer[i*244 + j*3 + 5] & 0xFC)<<3)
						  + ((buffer[i*244 + j*3 + 6] & 0xF8)>>3);
			image[i*80+j] = __LEu16(&image[i*80+j]);
			/*image[i*80+j] =((buffer[i*240 + j*3 + 2] & 0xF8)<<8)
						  + ((buffer[i*240 + j*3] & 0xF8)<<2)
						  + ((buffer[i*240 + j*3 + 1] & 0xF8)>>3);*/
		}
	}

	ili9341_draw_image(_lcd, 0, 0, 320, 240, image);

}

uint8_t* lepton_read_image(ili9341_t* _lcd){
	uint8_t buffer[60*160];
	uint16_t image[60*80];
	uint8_t header[4];
	HAL_SPI_Receive(HSPI, (uint8_t*)header, 4, 1000);
	uint16_t IDs[60];


	while((header[0] & 0x0F) == 0x0F){
		HAL_SPI_Receive(HSPI, (uint8_t*)buffer, 160, 1000);
		HAL_SPI_Receive(HSPI, (uint8_t*)header, 4, 1000);
	}

	HAL_SPI_Receive(HSPI, (uint8_t*)buffer, 160, 1000);

	for(int i = 1; i < 60; i++){
		if((header[0] & 0x0F) == 0x0F){
			SPI3_CS_GPIO_Port->ODR |= SPI3_CS_Pin;
			HAL_Delay(10);
			SPI3_CS_GPIO_Port->ODR &= ~SPI3_CS_Pin;
			HAL_Delay(10);
			return NULL;
		}
		IDs[i] = header[1];
		HAL_SPI_Receive(HSPI, (uint8_t*)header, 4, 1000);
		HAL_SPI_Receive(HSPI, (uint8_t*)&buffer[i*160], 160, 1000);
	}

	for(int i = 0; i < 60; i++){
		for(int j = 0; j < 80; j++){
			image[i*80+j] = buffer[i*160 + 2*j]<<8 + buffer[i*160 + 2*j + 1];
			//image[i*80+j] = __LEu16(&image[i*80+j]);
		}
	}

	ili9341_draw_image(_lcd, 0, 0, 320, 240, image);

	return buffer;
}

void lepton_AGC_enable(){
	uint8_t data[4] = {0, 1, 0, 0};
	lepton_write(0x0101, data, 2);
}

void lepton_AGC_disable(){
	uint8_t data[4] = {0, 0, 0, 0};
	lepton_write(0x0101, data, 2);
}

uint16_t lepton_read_AGC_state(){
	uint16_t data[2];
	lepton_read(0x0100, data, 2);

	return data[0];
}

void lepton_AGC_policy_select(uint8_t policy){
	uint8_t data[4] = {0, policy, 0, 0};
	lepton_write(0x0105, data, 2);
}

void lepton_AGC_policy_get(){
	uint16_t data[2];
	lepton_read(0x0104, data, 2);

	return data[0];
}

void lepton_AGC_ROI_select(uint8_t start_col, uint8_t start_row, uint8_t end_col, uint8_t end_row){
	uint8_t data[8] = {0, start_col, 0, start_row, 0, end_col, 0, end_row};
	lepton_write(0x0109, data, 4);
}

void lepton_AGC_histogram_stats(uint16_t* data){
	lepton_read(0x010C, data, 4);
}

void lepton_AGC_HEQ_damp_fact_set(uint16_t damp_fact){
	uint8_t data[2] = {damp_fact >> 8, damp_fact & 0xFF};
	lepton_write(0x0125, data, 1);
}

uint16_t lepton_AGC_HEQ_damp_fact_get(){
	uint8_t data[1];
	lepton_read(0x0124, data, 1);

	return data[0];
}

void lepton_AGC_CLH_set(uint16_t CLH){
	uint8_t data[2] = {CLH & 0xFF, CLH>>8};
	lepton_write(0x012D, data, 1);
}

uint16_t lepton_AGC_CLH_get(){
	uint8_t data[1];
	lepton_write(0x012C, data, 1);

	return data[0];
}

void lepton_AGC_CLL_set(uint16_t CLL){
	uint8_t data[2] = {CLL & 0xFF, CLL>>8};
	lepton_write(0x0131, data, 1);
}

uint16_t lepton_AGC_CLL_get(){
	uint16_t data[1];
	lepton_read(0x0130, data, 1);

	return data[0];
}

void lepton_AGC_HEQ_empty_counts_set(uint16_t empty_counts){
	uint8_t data[2] = {empty_counts >> 8, empty_counts & 0xFF};
	lepton_write(0x013D, data, 1);
}

uint16_t lepton_AGC_HEQ_empty_counts_get(){
	uint16_t data[1];
	lepton_read(0x013C, data, 1);

	return data[0];
}

void lepton_AGC_scale_fact_set(uint8_t scale_fact){
	uint8_t data[4] = {0, scale_fact, 0, 0};
	lepton_write(0x0145, data, 2);
}

uint16_t lepton_AGC_scale_fact_get(){
	uint16_t data[2];
	lepton_read(0x0144, data, 2);

	return data[0];
}

void lepton_AGC_calc_enable(){
	uint8_t data[4] = {0, 1, 0, 0};
	lepton_write(0x0149, data, 2);
}

void lepton_AGC_calc_disable(){
	uint8_t data[4] = {0, 0, 0, 0};
	lepton_write(0x0149, data, 2);
}

uint16_t lepton_AGC_calc_enable_state_get(){
	uint16_t data[2];
	lepton_read(0x0148, data, 2);

	return data[0];
}

void lepton_AGC_linear_percent_set(uint8_t percent){
	uint8_t data[2] = {0, percent};
	lepton_write(0x014D, data, 1);
}

uint16_t lepton_AGC_linear_percent_get(){
	uint16_t data[1];
	lepton_read(0x014C, data, 1);

	return data[0];
}

uint16_t lepton_SYS_ping(){
	return lepton_run_command(0x0202);
}

void lepton_SYS_status_get(uint16_t* status_buffer){
	lepton_read(0x0204, status_buffer, 4);
}

uint64_t lepton_SYS_serial_number_get(){
	uint16_t data[4];
	lepton_read(0x0208, data, 4);

	return (data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3];
}

uint32_t lepton_SYS_camera_uptime_get(){
	uint16_t data[2];
	lepton_read(0x020C, data, 2);

	return (data[0]<<8) + data[1];
}

uint16_t lepton_SYS_aux_temp_get(){
	uint16_t data[1];
	lepton_read(0x0210, data, 1);

	return data[0];
}

uint16_t lepton_SYS_FPA_temp_get(){
	uint16_t data[1];
	lepton_read(0x0214, data, 1);

	return data[0];
}

void lepton_SYS_telemetry_disable(){
	uint8_t data[4] = {0, 0, 0, 0};
	lepton_write(0x0219, data, 2);
}

void lepton_SYS_telemetry_enable(){
	uint8_t data[4] = {0, 1, 0, 0};
	lepton_write(0x0219, data, 2);
}

uint16_t lepton_SYS_telemetry_enable_state_get(){
	uint16_t data[2];
	lepton_read(0x0218, data, 2);

	return data[0];
}

void lepton_SYS_telemetry_location_set(uint8_t location){
	uint8_t data[4] = {0, location, 0, 0};
	lepton_write(0x021D, data, 2);
}

uint16_t lepton_SYS_telemetry_location_get(){
	uint16_t data[2];
	lepton_read(0x021C, data, 2);

	return data[0];
}

void lepton_SYS_frame_average(){
	lepton_run_command(0x0222);
}

void lepton_SYS_frame_average_set(uint8_t frames){
	uint8_t data[4] = {0, frames, 0, 0};
	lepton_write(0x0225, data, 2);
}

void lepton_SYS_frame_average_get(){
	uint16_t data[2];
	lepton_read(0x0224, data, 2);

	return data[0];
}

void lepton_SYS_customer_serial_number_get(uint16_t* buffer){
	lepton_read(0x0228, buffer, 16);
}

void lepton_SYS_ROI_select(uint8_t start_col, uint8_t start_row, uint8_t end_col, uint8_t end_row){
	uint8_t data[8] = {0, start_col, 0, start_row, 0, end_col, 0, end_row};
	lepton_write(0x0231, data, 2);
}

void lepton_SYS_shutter_close(){
	uint8_t data[4] = {0, 2, 0, 0};
	lepton_write(0x0239, data, 2);
}

void lepton_SYS_shutter_open(){
	uint8_t data[4] = {0, 1, 0, 0};
	lepton_write(0x0239, data, 2);
}

void lepton_SYS_set_gain_mode(uint8_t gain_mode){
	uint8_t data[4] = {0, gain_mode, 0, 0};
	lepton_write(0x0249, data, 2);
}

void lepton_SYS_gain_object_set(uint8_t ROI_start_col, uint8_t ROI_start_row, uint8_t ROI_end_col, uint8_t ROI_end_row,
								uint16_t P_htl, uint16_t P_lth, uint16_t C_htl, uint16_t C_lth,
								uint16_t T_htl, uint16_t T_lth, uint16_t ROI_pop, uint16_t temp_en,
								uint16_t flux_th_low, uint16_t flux_th_high){
	uint8_t data[28] = {0, ROI_start_col, 0, ROI_start_row, 0, ROI_end_col, 0, ROI_end_row,
						P_htl >> 8, P_htl & 0xFF, P_lth >> 8, P_lth & 0xFF,
						};
	lepton_write(0x0251, data, 2);
}





void lepton_video_format_RGB888(){
	uint8_t data[4] = {0, 3, 0, 0};
	lepton_write(0x0331, data, 2);
	HAL_Delay(100);
	lepton_write(0x4829, data, 2);
}

void lepton_select_LUT(uint8_t LUT){
	uint8_t data[4] = {0, LUT, 0, 0};
	lepton_write(0x0305, data, 2);
}

void lepton_select_LG_LUT(uint8_t LUT){
	uint8_t data[4] = {0, LUT, 0, 0};
	lepton_write(0x0335, data, 2);
}

void lepton_AGC_set_FMT(uint32_t FMT){
	uint8_t data[4] = {(FMT & 0x0000FF00)>>8, FMT & 0x000000FF, (FMT & 0xFF000000)>>24, (FMT & 0x00FF0000)>>16};
	lepton_write(0x0315, data, 2);
}



void lepton_VID_focus_enable(){
	uint8_t data[4] = {0, 1, 0, 0};
	lepton_write(0x030D, data, 2);
}



void lepton_OEM_bad_pixel_replacement(){
	uint8_t data[4] = {0, 0, 0, 0};
	lepton_write(0x486D, data, 2);
}





void lepton_FFC_run(){
	lepton_write_command(0x0242);
}

void lepton_SYS_gain_mode(){
	uint8_t data[28] = {0x26, 0x94, 0x22, 0x3D, 0, 1,
						0x12, 0xC0, 0, 40, 0, 40,
						0, 40, 0, 40, 0, 50, 0, 40,
						0, 59, 0, 79, 0, 0, 0, 0};
	lepton_write(0x0251, data, 14);
}

void lepton_OEM_source_select(){
	uint8_t data[4] = {0, 1, 0, 0};
	lepton_write(0x482D, data, 2);
}

void lepton_OEM_output_enable(){
	uint8_t data[4] = {0, 1, 0, 0};
	lepton_write(0x4825, data, 2);
}

void lepton_OEM_FFC_norm_target(){
	uint8_t data[2] = {0x20, 0x00};
	lepton_write(0x4845, data, 1);
}



void lepton_VID_FMC_disable(){
	uint8_t data[4] = {0, 0, 0, 0};
	lepton_write(0x030D, data, 2);
}



void lepton_OEM_FFC_norm_target_set(uint16_t target){
	uint8_t data[2] = {target>>8, target & 0xFF};
	lepton_write(0x4845, data, 1);
}

void lepton_OEM_FFC_norm_target_run(){
	lepton_write_command(0x4846);
}

void lepton_OEM_temporal_filter_disable(){
	uint8_t data[4] = {0, 0, 0, 0};
	lepton_write(0x4871, data, 2);
}

void lepton_RAD_enable(){
	uint8_t data[4] = {0, 1, 0, 0};
	lepton_write(0x4E11, data, 2);
}

void lepton_RAD_disable(){
	uint8_t data[4] = {0, 0, 0, 0};
	lepton_write(0x4E11, data, 2);
}

void lepton_RAD_Tlin_enable(){
	uint8_t data[4] = {0, 1, 0, 0};
	lepton_write(0x4EC1, data, 2);
}

void lepton_RAD_set_linear_params(uint16_t emissivity, uint16_t TBkgK, uint16_t tauWindow, uint16_t TWindowK, uint16_t tauAtm, uint16_t TAtmK, uint16_t refWindow, uint16_t TRefK){
	uint8_t data[16] = {TRefK >> 8, TRefK & 0xFF, refWindow >> 8, refWindow & 0xFF,
						tauAtm >> 8, tauAtm & 0xFF, TAtmK >> 8, TAtmK & 0xFF,
						TWindowK >> 8, TWindowK & 0xFF, tauWindow >> 8, tauWindow & 0xFF,
						TBkgK >> 8, TBkgK & 0xFF, emissivity >> 8, emissivity & 0xFF};
	lepton_write(0x4EC1, data, 8);
}

void lepton_focus_disable(){
	uint8_t data[4] = {0, 0, 0, 0};
	lepton_write(0x030D, data, 2);
}



void lepton_RAD_set_ROI(){
	uint8_t data[8] = {0, 0, 0, 0, 0, 59, 0, 79};


	lepton_write(0x4ECD, data, 4);
}

void lepton_read_spotmeter(uint16_t* data){
	lepton_read(0x4ED0, data, 4);
}

