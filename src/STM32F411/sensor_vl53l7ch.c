#include "sensor_vl53l7ch.h"
#include <stdio.h>


uint8_t sensor_vl53l7ch_init(VL53L7CX_Configuration *dev) {
    uint8_t status, isAlive;
    uint32_t 				integration_time_ms;

    dev->platform.address = VL53L7CX_DEFAULT_I2C_ADDRESS;

    // Check if the sensor is alive
    status = vl53l7cx_is_alive(dev, &isAlive);
    if (!isAlive || status) {
    	printf("VL53L7CX not detected at requested address\n");
        return status;
    }

    // Initialize the sensor
    status = vl53l7cx_init(dev);
    if (status) {
    	printf("VL53L7CX ULD Loading failed\n");
        return status;
    }

    printf("VL53L7CX ULD ready! (Version: %s)\n", VL53L7CX_API_REVISION);


    /*********************************/
	/*        Set some params        */
	/*********************************/
    status = vl53l7cx_set_ranging_mode(dev,  VL53L7CX_RANGING_MODE_AUTONOMOUS);
	if(status)
	{
		printf("vl53l7cx_set_ranging_mode failed, status %u\n", status);
		return status;
	}


	/* Set resolution in 8x8. WARNING : As others settings depend to this
	 * one, it must be the first to use.
	 */
	status = vl53l7cx_set_resolution(dev, VL53L7CX_RESOLUTION_8X8);
	if(status)
	{
		printf("vl53l7cx_set_resolution failed, status %u\n", status);
		return status;
	}

	/* Set ranging frequency to 10Hz.
	 * Using 4x4, min frequency is 1Hz and max is 60Hz
	 * Using 8x8, min frequency is 1Hz and max is 15Hz
	 */
	status = vl53l7cx_set_ranging_frequency_hz(dev, 10);
	if(status)
	{
		printf("vl53l7cx_set_ranging_frequency_hz failed, status %u\n", status);
		return status;
	}

	/* Set target order to closest */
	status = vl53l7cx_set_target_order(dev, VL53L7CX_TARGET_ORDER_CLOSEST);
	if(status)
	{
		printf("vl53l7cx_set_target_order failed, status %u\n", status);
		return status;
	}

	/* Get current integration time */

	status = vl53l7cx_get_integration_time_ms(dev, &integration_time_ms);
	if(status)
	{
		printf("vl53l7cx_get_integration_time_ms failed, status %u\n", status);
		return status;
	}
	printf("Current integration time is : %d ms\n", (int)integration_time_ms);


    return 0;

}

uint8_t sensor_vl53l7ch_start_ranging(VL53L7CX_Configuration *dev) {
    return vl53l7cx_start_ranging(dev);
}

uint8_t sensor_vl53l7ch_check_data_ready(VL53L7CX_Configuration *dev, uint8_t *isReady) {
    return vl53l7cx_check_data_ready(dev, isReady);
}

uint8_t sensor_vl53l7ch_get_ranging_data(VL53L7CX_Configuration *dev, VL53L7CX_ResultsData *results) {
    return vl53l7cx_get_ranging_data(dev, results);
}

uint8_t sensor_vl53l7ch_stop_ranging(VL53L7CX_Configuration *dev) {
    return vl53l7cx_stop_ranging(dev);
}

void sensor_vl53l7ch_print_data(VL53L7CX_Configuration *dev, VL53L7CX_ResultsData *results) {
    for (uint8_t i = 0; i < 64; i++) {
        printf("Zone: %3d, Status: %3u, Distance: %4d mm\n",
               i,
			   results->target_status[VL53L7CX_NB_TARGET_PER_ZONE*i],
			   results->distance_mm[VL53L7CX_NB_TARGET_PER_ZONE*i]);
    }
}

uint8_t sensor_vl53l7ch_WaitMs(VL53L7CX_Configuration *dev) {
	VL53L7CX_WaitMs(&(dev->platform), 5);
    return 0;
}
