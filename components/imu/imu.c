#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "imu.h"
struct bmi160_dev sensor;
int8_t imu_bmi160_user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length, void* cb_user_data)
{
    imu_t* imu = (imu_t*)cb_user_data;

    if (length == 0) {
        return (int8_t)ESP_ERR_INVALID_SIZE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_ACK_CHECK_DIS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, I2C_ACK_CHECK_EN);
    if (length > 1) {
        i2c_master_read(cmd, reg_data, length - 1, I2C_ACK_VAL);
    }
    i2c_master_read_byte(cmd, reg_data+length-1, I2C_NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(imu->i2c.port_number, cmd, IMU_I2C_MAX_WAIT);
    i2c_cmd_link_delete(cmd);
    return (int8_t)ret;
}

int8_t imu_bmi160_user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length, void* cb_user_data)
{	
    imu_t* imu = (imu_t*)cb_user_data;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_ACK_CHECK_EN);
    i2c_master_write(cmd, reg_data, length, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(imu->i2c.port_number, cmd, IMU_I2C_MAX_WAIT);
    i2c_cmd_link_delete(cmd);
    return (int8_t)ret;
}

void imu_bmi160_user_delay_ms(uint32_t period)
{
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    do {
        // do nothing
    } while((xTaskGetTickCount() * portTICK_PERIOD_MS) - start <= period);
}

int8_t imu_bmm150_user_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len, void* cb_user_data)
{
    imu_t* imu = (imu_t*)cb_user_data;

    return bmi160_aux_read(reg_addr, aux_data, len, &imu->sensor_bmi160);
}

int8_t imu_bmm150_user_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len, void* cb_user_data)
{
    imu_t* imu = (imu_t*)cb_user_data;

	return bmi160_aux_write(reg_addr, aux_data, len, &imu->sensor_bmi160);
}

void imu_bmm150_user_delay_ms(uint32_t period)
{
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    do {
        // do nothing
    } while((xTaskGetTickCount() * portTICK_PERIOD_MS) - start <= period);
}

void imu_init_i2c(imu_t *imu)
{
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = imu->i2c.pins.sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = imu->i2c.pins.scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = imu->i2c.frequency_hz;

    i2c_param_config(imu->i2c.port_number, &conf);
    i2c_driver_install(imu->i2c.port_number, conf.mode, 0, 0, 0);
}
void init_i2c()
{
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}
void initSensor()
{
    sensor.id = BMI160_I2C_ADDR;
    sensor.interface = BMI160_I2C_INTF;
    sensor.read = imu_bmi160_user_i2c_read;
    sensor.write = imu_bmi160_user_i2c_write;
    sensor.delay_ms = imu_bmi160_user_delay_ms;
    int8_t rslt = BMI160_OK;
    rslt = bmi160_init(&sensor);
    printf("init %d\n", rslt); // rslt is 0, so it seems work
}
void confAccel()
{
    /* Select the Output data rate, range of accelerometer sensor */
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_8G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    /* Select the power mode of accelerometer sensor */
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
    int8_t rslt = BMI160_OK;
    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&sensor);
    printf("conf %d\n", rslt); // rslt is 0, so it seems work
}
void selfTestAccel()
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr = 0x40;
    uint8_t data = 0x2C;
    uint16_t len = 1;
    rslt = bmi160_set_regs(reg_addr, &data, len, &sensor); //  rslt is 0, so it seems work
    rslt = bmi160_perform_self_test(BMI160_ACCEL_ONLY, &sensor); // rslt is 0, so it seems work
}
void readAccel()
{
	struct bmi160_sensor_data accel;
    int8_t rslt = BMI160_OK;
    rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &sensor);
    //int8_t rslt;
    uint8_t data = 0;
    	//float ax = convert_raw_acceleration(accel.x);
        //float ay = convert_raw_acceleration(accel.y);
        //float az = convert_raw_acceleration(accel.z);
        //float gx = convert_raw_gyro(gyro.x);
        //float gy = convert_raw_gyro(gyro.y);
        //float gz = convert_raw_gyro(gyro.z);

        ESP_LOGI(TAG, "[ACCEL] X: %d, Y: %d, Z: %d", accel.x, accel.y, accel.z);
        //ESP_LOGI(TAG, "[GYRO]  X: %f, Y: %f, Z: %f", gx, gy, gz);
    /* Read the error reg */
   //rslt = bmi160_get_regs(BMI160_ERROR_REG_ADDR, &data, 1, &sensor);

    if (rslt == BMI160_OK) {
    	data = data >> 1;
    	data = data & BMI160_ERR_REG_MASK;
    	if (data == 1)
    		rslt = BMI160_E_ACCEL_ODR_BW_INVALID;
    	else if (data == 2)
    		rslt = BMI160_E_GYRO_ODR_BW_INVALID;
    	else if (data == 3)
    		rslt = BMI160_E_LWP_PRE_FLTR_INT_INVALID;
    	else if (data == 7)
    	rslt = BMI160_E_LWP_PRE_FLTR_INVALID;
    }

    printf("Error register's rslt: %d\n", rslt);
}
void imu_init_bmi160(imu_t *imu)
{
    int8_t result;

    imu->sensor_bmi160.id = imu->i2c.bmi160_address;
    imu->sensor_bmi160.interface = BMI160_I2C_INTF;
    imu->sensor_bmi160.read = imu_bmi160_user_i2c_read;
    imu->sensor_bmi160.write = imu_bmi160_user_i2c_write;
    imu->sensor_bmi160.delay_ms = imu_bmi160_user_delay_ms;
    imu->sensor_bmi160.cb_user_data = (void*)imu;
    
    if ((result = bmi160_init(&imu->sensor_bmi160)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_init failed: %d", result);
        return;
    }
    ESP_LOGI(TAG, "BMI160 CHIP ID : %d", imu->sensor_bmi160.chip_id);

    /* Select the Output data rate, range of accelerometer sensor */
    imu->sensor_bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_800HZ;
    imu->sensor_bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    imu->sensor_bmi160.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    imu->sensor_bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    imu->sensor_bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
    imu->sensor_bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    imu->sensor_bmi160.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    imu->sensor_bmi160.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    if ((result = bmi160_set_sens_conf(&imu->sensor_bmi160)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_set_sens_conf failed: %d", result);
        return;
    }
}

void imu_init_perform_self_testing(imu_t *imu)
{
    // TODO: add support for self testing
}

void imu_init_bmm150_aux(imu_t *imu)
{
    int8_t result;

    imu->sensor_bmi160.aux_cfg.aux_sensor_enable = 1;
    imu->sensor_bmi160.aux_cfg.aux_i2c_addr = imu->i2c.bmm150_address;
    imu->sensor_bmi160.aux_cfg.manual_enable = 0;
    imu->sensor_bmi160.aux_cfg.aux_rd_burst_len = 2;

    imu->sensor_bmm150.read = imu_bmm150_user_read;
    imu->sensor_bmm150.write = imu_bmm150_user_write;
    imu->sensor_bmm150.delay_ms = imu_bmm150_user_delay_ms;
    imu->sensor_bmm150.dev_id = imu->i2c.bmm150_address;
    imu->sensor_bmm150.intf = BMM150_I2C_INTF;
    imu->sensor_bmm150.cb_user_data = (void*)imu;

    if ((result = bmi160_aux_init(&imu->sensor_bmi160)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_init failed: %d", result);
        return;
    }

    if ((result = bmm150_init(&imu->sensor_bmm150)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmm150_init failed: %d", result);
        return;
    }
    ESP_LOGI(TAG, "BMM150 CHIP ID : %d", imu->sensor_bmm150.chip_id);

    imu->sensor_bmm150.settings.pwr_mode = BMM150_NORMAL_MODE;
    if ((result = bmm150_set_op_mode(&imu->sensor_bmm150)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmm150_set_op_mode failed: %d", result);
        return;
    }

    imu->sensor_bmm150.settings.preset_mode = BMM150_PRESETMODE_LOWPOWER;
    if ((result = bmm150_set_presetmode(&imu->sensor_bmm150)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmm150_set_presetmode failed: %d", result);
        return;
    }

    imu->sensor_bmi160.aux_cfg.aux_odr = 8;
    if ((result = bmi160_config_aux_mode(&imu->sensor_bmi160)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_config_aux_mode failed: %d", result);
        return;
    }

    if ((result = bmi160_set_aux_auto_mode(&imu->sensor_bmi160.aux_cfg.aux_i2c_addr, &imu->sensor_bmi160)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_set_aux_auto_mode failed: %d", result);
        return;
    }
}

imu_t* imu_alloc(uint8_t i2c_bmi160_address, uint8_t i2c_bmm150_address, gpio_num_t i2c_pins_sda, 
                 gpio_num_t i2c_pins_scl, uint8_t i2c_port_number, uint32_t frequency_hz)
{
    imu_t *imu = (imu_t*)malloc(sizeof(imu_t));
    memset(imu, 0, sizeof(imu_t));

    imu->i2c.bmi160_address = i2c_bmi160_address;
    imu->i2c.bmm150_address = i2c_bmm150_address;
    imu->i2c.port_number = i2c_port_number;
    imu->i2c.frequency_hz = frequency_hz;
    imu->i2c.pins.sda = i2c_pins_sda;
    imu->i2c.pins.scl = i2c_pins_scl;

    return imu;
}

void imu_init(imu_t *imu)
{
    imu_init_i2c(imu);
    imu_init_bmi160(imu);
    imu_init_perform_self_testing(imu);
    //imu_init_bmm150_aux(imu);
}

void imu_free(imu_t *imu)
{
    free(imu);
}
