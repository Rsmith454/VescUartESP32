#include <Arduino.h>
#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;

void setup() {

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial.begin(115200);
  
  while (!Serial) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial);
}

void loop() {
  
  /** Call the function getVescValues() to acquire data from VESC */
  uint8_t data[1] = {COMM_GET_APPCONF};
  if (UART.getVescValues(data)) {

    Serial.println("data recived start:");
    Serial.print("controller_id:");
    Serial.println(UART.appconf.controller_id);
    Serial.println(UART.appconf.timeout_msec);
    Serial.println(UART.appconf.timeout_brake_current);
	Serial.println(UART.appconf.can_baud_rate); 
    Serial.println(UART.appconf.app_to_use); 
    Serial.print("app_ppm_conf.ctrl_type:");
	Serial.println(UART.appconf.app_ppm_conf.ctrl_type); 
	Serial.println(UART.appconf.app_ppm_conf.pid_max_erpm); 
	Serial.println(UART.appconf.app_ppm_conf.hyst);
	Serial.println(UART.appconf.app_ppm_conf.pulse_start );
	Serial.println(UART.appconf.app_ppm_conf.pulse_end );
	Serial.println(UART.appconf.app_ppm_conf.pulse_center );
	Serial.println(UART.appconf.app_ppm_conf.median_filter );
	Serial.println(UART.appconf.app_ppm_conf.safe_start );
	Serial.println(UART.appconf.app_ppm_conf.throttle_exp );
	Serial.println(UART.appconf.app_ppm_conf.throttle_exp_brake );
	Serial.println(UART.appconf.app_ppm_conf.throttle_exp_mode );
	Serial.println(UART.appconf.app_ppm_conf.ramp_time_pos );
	Serial.println(UART.appconf.app_ppm_conf.ramp_time_neg );
	Serial.println(UART.appconf.app_ppm_conf.multi_esc );
	Serial.println(UART.appconf.app_ppm_conf.tc  );
	Serial.println(UART.appconf.app_ppm_conf.tc_max_diff );
    Serial.print("UART.appconf.app_adc_conf.ctrl_type:");
	Serial.println(UART.appconf.app_adc_conf.ctrl_type );
	Serial.println(UART.appconf.app_adc_conf.hyst );
	Serial.println(UART.appconf.app_adc_conf.voltage_start );
	Serial.println(UART.appconf.app_adc_conf.voltage_end );
	Serial.println(UART.appconf.app_adc_conf.voltage_center );
	Serial.println(UART.appconf.app_adc_conf.voltage2_start );
	Serial.println(UART.appconf.app_adc_conf.voltage2_end );
	Serial.println(UART.appconf.app_adc_conf.use_filter );
	Serial.println(UART.appconf.app_adc_conf.safe_start );
	Serial.println(UART.appconf.app_adc_conf.voltage_inverted );
	Serial.println(UART.appconf.app_adc_conf.voltage2_inverted );
	Serial.println(UART.appconf.app_adc_conf.throttle_exp );
	Serial.println(UART.appconf.app_adc_conf.throttle_exp_brake );
	Serial.println(UART.appconf.app_adc_conf.throttle_exp_mode );
	Serial.println(UART.appconf.app_adc_conf.ramp_time_pos );
	Serial.println(UART.appconf.app_adc_conf.ramp_time_neg );
	Serial.println(UART.appconf.app_adc_conf.multi_esc );
	Serial.println(UART.appconf.app_adc_conf.tc  );
	Serial.println(UART.appconf.app_adc_conf.tc_max_diff );
	Serial.println(UART.appconf.app_adc_conf.update_rate_hz );
	Serial.println(UART.appconf.app_uart_baudrate );
    Serial.print("UART.appconf.app_chuk_conf.ctrl_type");
	Serial.println(UART.appconf.app_chuk_conf.ctrl_type );
	Serial.println(UART.appconf.app_chuk_conf.hyst );
	Serial.println(UART.appconf.app_chuk_conf.ramp_time_pos );
	Serial.println(UART.appconf.app_chuk_conf.ramp_time_neg );
	Serial.println(UART.appconf.app_chuk_conf.stick_erpm_per_s_in_cc );
	Serial.println(UART.appconf.app_chuk_conf.throttle_exp );
	Serial.println(UART.appconf.app_chuk_conf.throttle_exp_brake); 
	Serial.println(UART.appconf.app_chuk_conf.throttle_exp_mode); 
	Serial.println(UART.appconf.app_chuk_conf.multi_esc); 
	Serial.println(UART.appconf.app_chuk_conf.tc); 
	Serial.println(UART.appconf.app_chuk_conf.tc_max_diff); 
    Serial.print("UART.appconf.app_nrf_conf.speed");
	Serial.println(UART.appconf.app_nrf_conf.speed); 
	Serial.println(UART.appconf.app_nrf_conf.power); 
	Serial.println(UART.appconf.app_nrf_conf.crc_type); 
	Serial.println(UART.appconf.app_nrf_conf.retry_delay); 
	Serial.println(UART.appconf.app_nrf_conf.retries); 
	Serial.println(UART.appconf.app_nrf_conf.channel); 
	Serial.println(UART.appconf.app_nrf_conf.send_crc_ack); 
  }
  else
  {
    Serial.println("Failed to get data!");
  }

  delay(1000);
}