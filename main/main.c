#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

//Define GPIO for i2c peripheral
#define SDA_PIN_W 	GPIO_NUM_16
#define SCL_PIN_W	GPIO_NUM_13

//max_m10s i2c device address
#define I2C_ADD 0x42

typedef struct {
    int utc_hour;
    int utc_minute;
    float utc_second;
    float latitude;
    char lat_N_S;
    float longitude;
    char lon_E_W;
    int fix_quality;
    int num_satellites;
    float horizontal_dilution;
    float altitude;
    char alt_unit;
    float height_geoid;
    char height_geoid_unit;
} NMEAGGA_Data_s;

//Define NMEA GGA data structure
NMEAGGA_Data_s GGA_data;

void parseNMEA_GGA (const char* nmeaGgaString, NMEAGGA_Data_s* GGA_data){


    sscanf(nmeaGgaString,"$GNGGA,%2d%2d%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f,%c",
           &GGA_data->utc_hour, &GGA_data->utc_minute, &GGA_data->utc_second,
           &GGA_data->latitude, &GGA_data->lat_N_S,
           &GGA_data->longitude, &GGA_data->lon_E_W,
           &GGA_data->fix_quality, &GGA_data->num_satellites, &GGA_data->horizontal_dilution,
           &GGA_data->altitude, &GGA_data->alt_unit, &GGA_data->height_geoid,
           &GGA_data->height_geoid_unit);

    //Adjust the latitude and longitude to decimal representation
    GGA_data->latitude = (int)(GGA_data->latitude / 100) +fmod(GGA_data->latitude, 100) /60.0;
    if (GGA_data->lat_N_S == 'S') GGA_data->latitude = -GGA_data->latitude;

    GGA_data->longitude = (int)(GGA_data->longitude / 100) +fmod(GGA_data->longitude, 100) /60.0;
    if (GGA_data->lon_E_W == 'W') GGA_data->longitude = -GGA_data->longitude;

}

void printGGA_data(NMEAGGA_Data_s GGA_data){

	printf("UTC: %d:%d:%.0f\n",GGA_data.utc_hour, GGA_data.utc_minute, GGA_data.utc_second);
	printf("Latitude: %f  Longitude: %f  Altitude: %.2f m\n",GGA_data.latitude, GGA_data.longitude, GGA_data.altitude);
	printf("Fix: %d  N of satellites: %d\n",GGA_data.fix_quality, GGA_data.num_satellites);
	printf("Horizontal dilution: %.2f  Height geoid: %.2f %c\n\n",GGA_data.horizontal_dilution,GGA_data.height_geoid, GGA_data.height_geoid_unit);

}

// Function to calculate checksum
char calculateChecksum(const char* inString) {
    unsigned char checksum = 0;
    //Discard the first character '$' - Counts up to the character '*'
    for (const char* p = inString + 1; *p != '*' && *p != '\0'; p++) {
        checksum ^= *p;
    }
    return checksum;
}

// Function to validate the checksum
bool validateChecksum(const char* nmeaString) {
    // Find the position of the '*' character
    const char* checksumPos = strchr(nmeaString, '*');
    if (checksumPos == NULL || checksumPos - nmeaString < 3) {
        return false;  // No '*' found or invalid string length
    }

    // Calculate the checksum
    char calculatedChecksum = calculateChecksum(nmeaString);

    // Extract the checksum from the string
    unsigned int receivedChecksum;
    sscanf(checksumPos + 1, "%2x", &receivedChecksum);

    // Compare the calculated checksum with the received checksum
    return calculatedChecksum == receivedChecksum;
}

void i2c_config(void){

	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN_W,
		.scl_io_num = SCL_PIN_W,
		.sda_pullup_en = GPIO_PULLUP_DISABLE,
		.scl_pullup_en = GPIO_PULLUP_DISABLE,
		.master.clk_speed = 400000,
	};
	i2c_param_config(I2C_NUM_0, &conf);

	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

static void GNSS_manager_task(void *arg){
	uint8_t rx_data[16];
	uint8_t tx_data[16];
	char gps_string_in[250];
	int string_idx = 0;
	uint32_t num_to_read = 0;

	tx_data[0] = 0xFD;

	while(1){
		i2c_master_write_read_device(I2C_NUM_0, I2C_ADD, tx_data, 1, rx_data, 2, 500/portTICK_PERIOD_MS);
		num_to_read = (uint32_t)rx_data[1] | ((uint32_t)rx_data[0] << 8);

		while(num_to_read > 0){
			i2c_master_write_read_device(I2C_NUM_0, I2C_ADD, tx_data, 1, rx_data, 3, 500/portTICK_PERIOD_MS);
			gps_string_in[string_idx] = (char)rx_data[2];
			if(rx_data[2] == '\n'){
				gps_string_in[string_idx++] = '\0';
				if(strstr(gps_string_in, "GNGGA" ) != NULL){
					if(validateChecksum(gps_string_in) == true){
						parseNMEA_GGA(gps_string_in, &GGA_data);
						printGGA_data(GGA_data);
					}else{
						ESP_LOGI("NMEA", "Input string Checksum error!");
					}
				}
				string_idx = 0;
			}else{
				string_idx ++;
			}
			num_to_read = (uint32_t)rx_data[1] | ((uint32_t)rx_data[0] << 8);
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}

}


void app_main(void)
{
	i2c_config();

	xTaskCreatePinnedToCore(GNSS_manager_task, "GNSS_manager", 4*1024, NULL, 5, NULL,1);

    while (true) {
    	vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
