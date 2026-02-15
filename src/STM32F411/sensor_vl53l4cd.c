#include "vl53l4cd_api.h"
#include "platform2.h"
#include <stdio.h>

Dev_t dev = 0x52; // Default I2C address

uint8_t sensor_vl53l4cd_init(void) {
    uint8_t status;
    uint16_t sensor_id;

    status = VL53L4CD_GetSensorId(dev, &sensor_id);
    if (status || sensor_id != 0xEBAA) {
        printf("VL53L4CD not detected at address 0x%X\n", dev);
        return status;
    }

    status = VL53L4CD_SensorInit(dev);
    if (status) {
        printf("VL53L4CD initialization failed\n");
        return status;
    }

    status = VL53L4CD_SetRangeTiming(dev, 50, 0); // 50ms timing budget, 0 intermeasurement (continuous)
    if (status) {
        printf("Failed to set range timing\n");
        return status;
    }

    printf("VL53L4CD ready!\n");
    return 0;
}

uint8_t sensor_vl53l4cd_start_ranging(void) {
    return VL53L4CD_StartRanging(dev);
}

uint8_t sensor_vl53l4cd_check_data_ready(uint8_t *isReady) {
    return VL53L4CD_CheckForDataReady(dev, isReady);
}

uint8_t sensor_vl53l4cd_get_ranging_data(VL53L4CD_ResultsData_t *results) {
    return VL53L4CD_GetResult(dev, results);
}

uint8_t sensor_vl53l4cd_stop_ranging(void) {
    return VL53L4CD_StopRanging(dev);
}

uint8_t sensor_vl53l4cd_clear_interrupt(void) {
    return VL53L4CD_ClearInterrupt(dev);
}

void sensor_vl53l4cd_print_data(VL53L4CD_ResultsData_t *results) {
    printf("Status = %u, Distance = %u mm, Signal = %u kcps/spad\n",
           results->range_status,
           results->distance_mm,
           results->signal_per_spad_kcps);
}
