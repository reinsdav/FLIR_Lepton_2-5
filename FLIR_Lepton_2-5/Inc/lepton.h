#include "main.h"
#include "ili9341.h"
#include "ili9341_gfx.h"

#define	HI2C  &hi2c1
#define HUART &huart2
#define HSPI  &hspi3

#define I2C_ADDRESS 0x2A<<1

typedef struct{
	uint8_t R;
	uint8_t G;
	uint8_t B;
} color_pixel_t;

const enum AGC_POLICY{
	AGC_POLICY_LINEAR,
	AGC_POLICY_HEQ
};

void lepton_write(uint16_t command, uint8_t* data, uint16_t size);
void lepton_read(uint16_t address, uint16_t* data, uint16_t size);
void lepton_read_data(uint16_t size, uint16_t* data);
void lepton_read_image_RGB(ili9341_t* _lcd);
void lepton_command_init();
void lepton_startup();
void lepton_write_length(uint16_t length);
void lepton_write_command(uint16_t command);

uint8_t* lepton_read_image(ili9341_t* _lcd);
uint16_t lepton_read_status();

void lepton_AGC_enable();
void lepton_AGC_disable();
uint16_t lepton_read_AGC_state();
void lepton_AGC_policy_select(uint8_t policy);
void lepton_AGC_ROI_select(uint8_t start_col, uint8_t start_row, uint8_t end_col, uint8_t end_row);
void lepton_video_format_RGB888();
void lepton_select_LUT(uint8_t LUT);
void lepton_AGC_set_FMT(uint32_t FMT);
void lepton_SYS_set_gain_mode(uint8_t gain_mode);
void lepton_select_LG_LUT(uint8_t LUT);
void lepton_AGC_HEQ_damp_fact_set(uint16_t damp_fact);
void lepton_VID_focus_enable();
void lepton_AGC_set_scale_fact();
void lepton_OEM_bad_pixel_replacement();
void lepton_AGC_HEQ_empty_count();
void lepton_SYS_shutter_close();
void lepton_SYS_shutter_open();
void lepton_FFC_run();
void lepton_SYS_gain_mode();
void lepton_OEM_source_select();
void lepton_OEM_FFC_norm_target();
void lepton_AGC_CLH_set(uint16_t CLH);
void lepton_AGC_CLL_set(uint16_t CLL);
uint16_t lepton_AGC_CLL_get();
void lepton_AGC_linear_percent(uint8_t percent);
void lepton_VID_FMC_disable();
uint16_t lepton_SYS_get_aux_temp();
void lepton_OEM_FFC_norm_target_set(uint16_t target);
void lepton_OEM_FFC_norm_target_run();
void lepton_OEM_temporal_filter_disable();
void lepton_RAD_enable();
void lepton_RAD_Tlin_enable();
void lepton_RAD_set_linear_params(uint16_t emissivity, uint16_t TBkgK, uint16_t tauWindow, uint16_t TWindowK, uint16_t tauAtm, uint16_t TAtmK, uint16_t refWindow, uint16_t TRefK);
void lepton_AGC_calc_enable();
void lepton_RAD_disable();
void lepton_focus_disable();
void lepton_OEM_output_enable();
void lepton_RAD_set_ROI();
void lepton_read_spotmeter(uint16_t* data);



