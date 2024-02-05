#include <stdint.h>
#include "VescUart.h"
#include "packet.h"

#define MCCONF_SIGNATURE		776184161
#define APPCONF_SIGNATURE		486554156

VescUart::VescUart(void){

}

void VescUart::setSerialPort(Stream* port){
	serialPort = port;
	this->packet.init( this->serialPort );
}

void VescUart::setDebugPort(Stream* port){
	debugPort = port;
}

int VescUart::receiveUartMessage(uint8_t * payloadReceived) {

	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	const uint16_t shortMessage = 256;
	const uint16_t longMessage = 512;

	uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[1024];
	uint16_t lenPayload = 0;
	bool isLongPacket = false;

	uint32_t timeout = millis() + 2500; // Defining the timestamp for timeout (100ms before timeout)

	while ( millis() < timeout && messageRead == false) {

		while (serialPort->available()) {

			messageReceived[counter] = serialPort->read();
			counter++;
			if (counter == 2) {

				switch (messageReceived[0])
				{
					case 2:
						endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
						lenPayload = messageReceived[1];
					break;

					case 3:
						isLongPacket = true;
						endMessage = ((messageReceived[1] << 8) | messageReceived[2]) + 6; //Payload size + 2 for sice + 3 for SRC and End.
						lenPayload = (messageReceived[1] << 8) | messageReceived[2];
					break;

					default:
						if( debugPort != NULL ){
							debugPort->println("Unvalid start bit");
						}
					break;
				}
			}

			if (counter >= sizeof(messageReceived)) {
				break;
			}

			if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
				messageReceived[endMessage] = 0;
				if (debugPort != NULL) {
					debugPort->println("End of message reached!");
				}
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}
	if(messageRead == false && debugPort != NULL ) {
		debugPort->println("Timeout");
		debugPort->printf("Counter: %d\n", counter);
	}
	
	bool unpacked = false;

	if (messageRead) {
		unpacked = (!isLongPacket) ? unpackPayload(messageReceived, endMessage, payloadReceived) : unpackPayload16(messageReceived, endMessage, payloadReceived);
	}

	if (unpacked) {
		// Message was read
		return lenPayload; 
	}
	else {
		// No Message Read
		return 0;
	}
}

bool VescUart::unpackPayload(uint8_t * message, int lenMes, uint8_t * payload) {

	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	if(debugPort!=NULL){
		debugPort->print("SRC received: "); debugPort->println(crcMessage);
	}

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if( debugPort != NULL ){
		debugPort->print("SRC calc: "); debugPort->println(crcPayload);
	}
	
	if (crcPayload == crcMessage) {
		if( debugPort != NULL ) {
			debugPort->print("Received: "); 
			serialPrint(message, lenMes); debugPort->println();

			debugPort->print("Payload :      ");
			serialPrint(payload, message[1] - 1); debugPort->println();
		}

		return true;
	}else{
		return false;
	}
}

bool VescUart::unpackPayload16(uint8_t * message, int lenMes, uint8_t * payload) {

	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	if(debugPort!=NULL){
		debugPort->print("SRC received: "); debugPort->println(crcMessage);
	}

	// Extract payload:
	memcpy(payload, &message[3], (message [1] << 8) | message[2]);
	crcPayload = crc16(payload,  (message[1] << 8) | message[2]);

	if( debugPort != NULL ){
		debugPort->print("SRC calc: "); debugPort->println(crcPayload);
	}
	
	if (crcPayload == crcMessage) {
		if( debugPort != NULL ) {
			debugPort->print("Received: "); 
			serialPrint(message, lenMes); debugPort->println();

			debugPort->print("Payload :      ");
			serialPrint(payload, message[1] - 1); debugPort->println();
		}

		return true;
	}else{
		return false;
	}
}

int VescUart::packSendPayload(uint8_t * payload, int lenPay) {

	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[512];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = '\0';

	if(debugPort!=NULL){
		debugPort->print("UART package send: "); serialPrint(messageSend, count);
	}

	// Sending package
	serialPort->write(messageSend, count);

	// Returns number of send bytes
	return count;
}

bool VescUart::processReadPacket(uint8_t * message) {

	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164
			data.tempMosfet 		= buffer_get_float16(message, 10.0, &ind);
			data.tempMotor 			= buffer_get_float16(message, 10.0, &ind);
			data.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &ind);
			data.avgInputCurrent 	= buffer_get_float32(message, 100.0, &ind);
			ind += 8; // Skip the next 8 bytes
			data.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &ind);
			data.rpm 				= buffer_get_float32(message, 1.0, &ind);
			data.inpVoltage 		= buffer_get_float16(message, 10.0, &ind);
			data.ampHours 			= buffer_get_float32(message, 10000.0, &ind);
			data.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &ind);
			data.wattHours			= buffer_get_float32(message, 10000.0, &ind);
			data.wattHoursCharged	= buffer_get_float32(message, 10000.0, &ind);
			data.tachometer 		= buffer_get_int32(message, &ind);
			data.tachometerAbs 		= buffer_get_int32(message, &ind);
			data.error 				= message[ind];
			return true;

		break;

		case COMM_GET_APPCONF:
			confgeneratorDeserializeAppconf(message, &appconf);
			return true;
		break;

		case COMM_GET_MCCONF:
			confgeneratorDeserializeMcconf(message, &mc_conf);
			return true;
		break;

		default:
			return false;
		break;
	}
}

bool VescUart::getVescValues(void) {

	uint8_t data[1] = { COMM_GET_VALUES };
	uint8_t payload[256];

	this->packet.send(data, sizeof(data));

	// packSendPayload(command, 1);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 55) {
		bool read = processReadPacket(payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

bool VescUart::getVescValues(uint8_t data[1]) {
	uint8_t payload[512];
	this->packet.send(data, sizeof(uint8_t));

	// packSendPayload(command, 1);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);
	Serial.println(lenPayload);

	if (lenPayload > 55) {
		bool read = processReadPacket(payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

/* The VESC remote app still uses the old nunchuck_app, so do we */
void VescUart::setRemoteData( uint8_t value, bool cruise, bool reverse ) {

	int32_t ind = 0;
	uint8_t payload[11];

	payload[ind++] = COMM_SET_CHUCK_DATA;
	payload[ind++] = value;
	payload[ind++] = 128; // Middle value
	buffer_append_bool(payload, cruise, &ind);
	buffer_append_bool(payload, reverse, &ind);
	
	// Acceleration Data. Not used, 3 * int16 (2 byte)
	for (size_t i = 0; i < 6; i++){
		payload[ind++] = 0;
	}

	this->packet.send(payload, sizeof(payload) );
}

void VescUart::setCurrent(float current) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setBrakeCurrent(float brakeCurrent) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setRPM(float rpm) {
	int32_t index = 0;
	uint8_t payload[512];

	payload[index++] = COMM_SET_RPM ;
	buffer_append_int32(payload, (int32_t)(rpm), &index);

	packSendPayload(payload, 5);
}

void VescUart::setDuty(float duty) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_DUTY;
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setAppConf(app_configuration *appconf){
	int32_t index = 0;
	uint8_t payload[512];
	payload[index++] = COMM_SET_APPCONF;
	uint32_t len = confgeneratorSerializeAppconf(payload + 1, appconf);
	debugPort->println("App len:");
	debugPort->println(len);
	packSendPayload(payload, len + 1);
}

void VescUart::setMcConf(mc_configuration *mc_conf){
	int32_t index = 0;
	uint8_t payload[512];
	payload[index++] = COMM_SET_MCCONF;
	uint32_t len = confgeneratorSerializeMcconf(payload + 1, mc_conf);
	debugPort->println("Mcconf len:");
	debugPort->println(len);
	packSendPayload(payload, len + 1);
}

void VescUart::serialPrint(uint8_t * data, int len) {
	if(debugPort != NULL){
		for (int i = 0; i <= len; i++)
		{
			debugPort->print(data[i]);
			debugPort->print(" ");
		}

		debugPort->println("");
	}
}

bool VescUart::confgeneratorDeserializeAppconf(const uint8_t *buffer, app_configuration *conf) {
	int32_t ind = 0;

	uint32_t signature = buffer_get_uint32(buffer, &ind);
	if (signature != APPCONF_SIGNATURE) {

		return false;
	}

	conf->controller_id = buffer[ind++];
	conf->timeout_msec = buffer_get_uint32(buffer, &ind);
	conf->timeout_brake_current = buffer_get_float32_auto(buffer, &ind);
	conf->can_status_rate_1 = buffer_get_uint16(buffer, &ind);
	conf->can_status_rate_2 = buffer_get_uint16(buffer, &ind);
	conf->can_status_msgs_r1 = buffer[ind++];
	conf->can_status_msgs_r2 = buffer[ind++];
	conf->can_baud_rate = (CAN_BAUD)buffer[ind++];
	conf->pairing_done = buffer[ind++];
	conf->permanent_uart_enabled = buffer[ind++];
	conf->shutdown_mode = (SHUTDOWN_MODE)buffer[ind++];
	conf->can_mode = (CAN_MODE)buffer[ind++];
	conf->uavcan_esc_index = buffer[ind++];
	conf->uavcan_raw_mode = (UAVCAN_RAW_MODE)buffer[ind++];
	conf->uavcan_raw_rpm_max = buffer_get_float32_auto(buffer, &ind);
	conf->uavcan_status_current_mode = (UAVCAN_STATUS_CURRENT_MODE)buffer[ind++];
	conf->servo_out_enable = buffer[ind++];
	conf->kill_sw_mode = (KILL_SW_MODE)buffer[ind++];
	conf->app_to_use = (app_use)buffer[ind++];
	conf->app_ppm_conf.ctrl_type = (ppm_control_type)buffer[ind++];
	conf->app_ppm_conf.pid_max_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.hyst = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.pulse_start = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.pulse_end = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.pulse_center = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.median_filter = buffer[ind++];
	conf->app_ppm_conf.safe_start = (SAFE_START_MODE)buffer[ind++];
	conf->app_ppm_conf.throttle_exp = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.throttle_exp_brake = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.throttle_exp_mode = (thr_exp_mode)buffer[ind++];
	conf->app_ppm_conf.ramp_time_pos = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.ramp_time_neg = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.multi_esc = buffer[ind++];
	conf->app_ppm_conf.tc = buffer[ind++];
	conf->app_ppm_conf.tc_max_diff = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.max_erpm_for_dir = buffer_get_float16(buffer, 1, &ind);
	conf->app_ppm_conf.smart_rev_max_duty = buffer_get_float32_auto(buffer, &ind);
	conf->app_ppm_conf.smart_rev_ramp_time = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.ctrl_type = (adc_control_type)buffer[ind++];
	conf->app_adc_conf.hyst = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.voltage_start = buffer_get_float16(buffer, 1000, &ind);
	conf->app_adc_conf.voltage_end = buffer_get_float16(buffer, 1000, &ind);
	conf->app_adc_conf.voltage_min = buffer_get_float16(buffer, 1000, &ind);
	conf->app_adc_conf.voltage_max = buffer_get_float16(buffer, 1000, &ind);
	conf->app_adc_conf.voltage_center = buffer_get_float16(buffer, 1000, &ind);
	conf->app_adc_conf.voltage2_start = buffer_get_float16(buffer, 1000, &ind);
	conf->app_adc_conf.voltage2_end = buffer_get_float16(buffer, 1000, &ind);
	conf->app_adc_conf.use_filter = buffer[ind++];
	conf->app_adc_conf.safe_start = (SAFE_START_MODE)buffer[ind++];
	conf->app_adc_conf.buttons = buffer[ind++];
	conf->app_adc_conf.voltage_inverted = buffer[ind++];
	conf->app_adc_conf.voltage2_inverted = buffer[ind++];
	conf->app_adc_conf.throttle_exp = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.throttle_exp_brake = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.throttle_exp_mode = (thr_exp_mode)buffer[ind++];
	conf->app_adc_conf.ramp_time_pos = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.ramp_time_neg = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.multi_esc = buffer[ind++];
	conf->app_adc_conf.tc = buffer[ind++];
	conf->app_adc_conf.tc_max_diff = buffer_get_float32_auto(buffer, &ind);
	conf->app_adc_conf.update_rate_hz = buffer_get_uint16(buffer, &ind);
	conf->app_uart_baudrate = buffer_get_uint32(buffer, &ind);
	conf->app_chuk_conf.ctrl_type = (chuk_control_type)buffer[ind++];
	conf->app_chuk_conf.hyst = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.ramp_time_pos = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.ramp_time_neg = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.stick_erpm_per_s_in_cc = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.throttle_exp = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.throttle_exp_brake = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.throttle_exp_mode = (thr_exp_mode)buffer[ind++];
	conf->app_chuk_conf.multi_esc = buffer[ind++];
	conf->app_chuk_conf.tc = buffer[ind++];
	conf->app_chuk_conf.tc_max_diff = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.use_smart_rev = buffer[ind++];
	conf->app_chuk_conf.smart_rev_max_duty = buffer_get_float32_auto(buffer, &ind);
	conf->app_chuk_conf.smart_rev_ramp_time = buffer_get_float32_auto(buffer, &ind);
	conf->app_nrf_conf.speed = (NRF_SPEED)buffer[ind++];
	conf->app_nrf_conf.power = (NRF_POWER)buffer[ind++];
	conf->app_nrf_conf.crc_type = (NRF_CRC)buffer[ind++];
	conf->app_nrf_conf.retry_delay = (NRF_RETR_DELAY)buffer[ind++];
	conf->app_nrf_conf.retries = (int8_t)buffer[ind++];
	conf->app_nrf_conf.channel = (int8_t)buffer[ind++];
	conf->app_nrf_conf.address[0] = buffer[ind++];
	conf->app_nrf_conf.address[1] = buffer[ind++];
	conf->app_nrf_conf.address[2] = buffer[ind++];
	conf->app_nrf_conf.send_crc_ack = buffer[ind++];
	conf->app_balance_conf.pid_mode = (BALANCE_PID_MODE)buffer[ind++];
	conf->app_balance_conf.kp = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.ki = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.kd = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.kp2 = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.ki2 = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.kd2 = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.hertz = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.loop_time_filter = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.fault_pitch = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.fault_roll = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.fault_duty = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.fault_adc1 = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.fault_adc2 = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.fault_delay_pitch = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.fault_delay_roll = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.fault_delay_duty = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.fault_delay_switch_half = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.fault_delay_switch_full = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.fault_adc_half_erpm = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.fault_is_dual_switch = buffer[ind++];
	conf->app_balance_conf.tiltback_duty_angle = buffer_get_float16(buffer, 100, &ind);
	conf->app_balance_conf.tiltback_duty_speed = buffer_get_float16(buffer, 100, &ind);
	conf->app_balance_conf.tiltback_duty = buffer_get_float16(buffer, 1000, &ind);
	conf->app_balance_conf.tiltback_hv_angle = buffer_get_float16(buffer, 100, &ind);
	conf->app_balance_conf.tiltback_hv_speed = buffer_get_float16(buffer, 100, &ind);
	conf->app_balance_conf.tiltback_hv = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.tiltback_lv_angle = buffer_get_float16(buffer, 100, &ind);
	conf->app_balance_conf.tiltback_lv_speed = buffer_get_float16(buffer, 100, &ind);
	conf->app_balance_conf.tiltback_lv = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.tiltback_return_speed = buffer_get_float16(buffer, 100, &ind);
	conf->app_balance_conf.tiltback_constant = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.tiltback_constant_erpm = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.tiltback_variable = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.tiltback_variable_max = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.noseangling_speed = buffer_get_float16(buffer, 100, &ind);
	conf->app_balance_conf.startup_pitch_tolerance = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.startup_roll_tolerance = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.startup_speed = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.deadzone = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.multi_esc = buffer[ind++];
	conf->app_balance_conf.yaw_kp = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.yaw_ki = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.yaw_kd = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.roll_steer_kp = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.roll_steer_erpm_kp = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.brake_current = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.brake_timeout = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.yaw_current_clamp = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.ki_limit = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.kd_pt1_lowpass_frequency = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.kd_pt1_highpass_frequency = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.booster_angle = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.booster_ramp = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.booster_current = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.torquetilt_start_current = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.torquetilt_angle_limit = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.torquetilt_on_speed = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.torquetilt_off_speed = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.torquetilt_strength = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.torquetilt_filter = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.turntilt_strength = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.turntilt_angle_limit = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.turntilt_start_angle = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.turntilt_start_erpm = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.turntilt_speed = buffer_get_float32_auto(buffer, &ind);
	conf->app_balance_conf.turntilt_erpm_boost = buffer_get_uint16(buffer, &ind);
	conf->app_balance_conf.turntilt_erpm_boost_end = buffer_get_uint16(buffer, &ind);
	conf->app_pas_conf.ctrl_type = (pas_control_type)buffer[ind++];
	conf->app_pas_conf.sensor_type = (pas_sensor_type)buffer[ind++];
	conf->app_pas_conf.current_scaling = buffer_get_float16(buffer, 1000, &ind);
	conf->app_pas_conf.pedal_rpm_start = buffer_get_float16(buffer, 10, &ind);
	conf->app_pas_conf.pedal_rpm_end = buffer_get_float16(buffer, 10, &ind);
	conf->app_pas_conf.invert_pedal_direction = buffer[ind++];
	conf->app_pas_conf.magnets = buffer_get_uint16(buffer, &ind);
	conf->app_pas_conf.use_filter = buffer[ind++];
	conf->app_pas_conf.ramp_time_pos = buffer_get_float16(buffer, 100, &ind);
	conf->app_pas_conf.ramp_time_neg = buffer_get_float16(buffer, 100, &ind);
	conf->app_pas_conf.update_rate_hz = buffer_get_uint16(buffer, &ind);
	conf->imu_conf.type = (IMU_TYPE)buffer[ind++];
	conf->imu_conf.mode = (AHRS_MODE)buffer[ind++];
	conf->imu_conf.filter = (IMU_FILTER)buffer[ind++];
	conf->imu_conf.accel_lowpass_filter_x = buffer_get_float16(buffer, 1, &ind);
	conf->imu_conf.accel_lowpass_filter_y = buffer_get_float16(buffer, 1, &ind);
	conf->imu_conf.accel_lowpass_filter_z = buffer_get_float16(buffer, 1, &ind);
	conf->imu_conf.gyro_lowpass_filter = buffer_get_float16(buffer, 1, &ind);
	conf->imu_conf.sample_rate_hz = buffer_get_uint16(buffer, &ind);
	conf->imu_conf.use_magnetometer = buffer[ind++];
	conf->imu_conf.accel_confidence_decay = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.mahony_kp = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.mahony_ki = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.madgwick_beta = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.rot_roll = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.rot_pitch = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.rot_yaw = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.accel_offsets[0] = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.accel_offsets[1] = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.accel_offsets[2] = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.gyro_offsets[0] = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.gyro_offsets[1] = buffer_get_float32_auto(buffer, &ind);
	conf->imu_conf.gyro_offsets[2] = buffer_get_float32_auto(buffer, &ind);

	return true;
}

int32_t VescUart::confgeneratorSerializeAppconf(uint8_t *buffer, const app_configuration *conf) {
	int32_t ind = 0;

	buffer_append_uint32(buffer, APPCONF_SIGNATURE, &ind);

	buffer[ind++] = (uint8_t)conf->controller_id;
	buffer_append_uint32(buffer, conf->timeout_msec, &ind);
	buffer_append_float32_auto(buffer, conf->timeout_brake_current, &ind);
	buffer_append_uint16(buffer, conf->can_status_rate_1, &ind);
	buffer_append_uint16(buffer, conf->can_status_rate_2, &ind);
	buffer[ind++] = conf->can_status_msgs_r1;
	buffer[ind++] = conf->can_status_msgs_r2;
	buffer[ind++] = conf->can_baud_rate;
	buffer[ind++] = conf->pairing_done;
	buffer[ind++] = conf->permanent_uart_enabled;
	buffer[ind++] = conf->shutdown_mode;
	buffer[ind++] = conf->can_mode;
	buffer[ind++] = (uint8_t)conf->uavcan_esc_index;
	buffer[ind++] = conf->uavcan_raw_mode;
	buffer_append_float32_auto(buffer, conf->uavcan_raw_rpm_max, &ind);
	buffer[ind++] = conf->uavcan_status_current_mode;
	buffer[ind++] = conf->servo_out_enable;
	buffer[ind++] = conf->kill_sw_mode;
	buffer[ind++] = conf->app_to_use;
	buffer[ind++] = conf->app_ppm_conf.ctrl_type;
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.pid_max_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.hyst, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.pulse_start, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.pulse_end, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.pulse_center, &ind);
	buffer[ind++] = conf->app_ppm_conf.median_filter;
	buffer[ind++] = conf->app_ppm_conf.safe_start;
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.throttle_exp, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.throttle_exp_brake, &ind);
	buffer[ind++] = conf->app_ppm_conf.throttle_exp_mode;
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.ramp_time_neg, &ind);
	buffer[ind++] = conf->app_ppm_conf.multi_esc;
	buffer[ind++] = conf->app_ppm_conf.tc;
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.tc_max_diff, &ind);
	buffer_append_float16(buffer, conf->app_ppm_conf.max_erpm_for_dir, 1, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.smart_rev_max_duty, &ind);
	buffer_append_float32_auto(buffer, conf->app_ppm_conf.smart_rev_ramp_time, &ind);
	buffer[ind++] = conf->app_adc_conf.ctrl_type;
	buffer_append_float32_auto(buffer, conf->app_adc_conf.hyst, &ind);
	buffer_append_float16(buffer, conf->app_adc_conf.voltage_start, 1000, &ind);
	buffer_append_float16(buffer, conf->app_adc_conf.voltage_end, 1000, &ind);
	buffer_append_float16(buffer, conf->app_adc_conf.voltage_min, 1000, &ind);
	buffer_append_float16(buffer, conf->app_adc_conf.voltage_max, 1000, &ind);
	buffer_append_float16(buffer, conf->app_adc_conf.voltage_center, 1000, &ind);
	buffer_append_float16(buffer, conf->app_adc_conf.voltage2_start, 1000, &ind);
	buffer_append_float16(buffer, conf->app_adc_conf.voltage2_end, 1000, &ind);
	buffer[ind++] = conf->app_adc_conf.use_filter;
	buffer[ind++] = conf->app_adc_conf.safe_start;
	buffer[ind++] = conf->app_adc_conf.buttons;
	buffer[ind++] = conf->app_adc_conf.voltage_inverted;
	buffer[ind++] = conf->app_adc_conf.voltage2_inverted;
	buffer_append_float32_auto(buffer, conf->app_adc_conf.throttle_exp, &ind);
	buffer_append_float32_auto(buffer, conf->app_adc_conf.throttle_exp_brake, &ind);
	buffer[ind++] = conf->app_adc_conf.throttle_exp_mode;
	buffer_append_float32_auto(buffer, conf->app_adc_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(buffer, conf->app_adc_conf.ramp_time_neg, &ind);
	buffer[ind++] = conf->app_adc_conf.multi_esc;
	buffer[ind++] = conf->app_adc_conf.tc;
	buffer_append_float32_auto(buffer, conf->app_adc_conf.tc_max_diff, &ind);
	buffer_append_uint16(buffer, conf->app_adc_conf.update_rate_hz, &ind);
	buffer_append_uint32(buffer, conf->app_uart_baudrate, &ind);
	buffer[ind++] = conf->app_chuk_conf.ctrl_type;
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.hyst, &ind);
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.ramp_time_neg, &ind);
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.stick_erpm_per_s_in_cc, &ind);
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.throttle_exp, &ind);
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.throttle_exp_brake, &ind);
	buffer[ind++] = conf->app_chuk_conf.throttle_exp_mode;
	buffer[ind++] = conf->app_chuk_conf.multi_esc;
	buffer[ind++] = conf->app_chuk_conf.tc;
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.tc_max_diff, &ind);
	buffer[ind++] = conf->app_chuk_conf.use_smart_rev;
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.smart_rev_max_duty, &ind);
	buffer_append_float32_auto(buffer, conf->app_chuk_conf.smart_rev_ramp_time, &ind);
	buffer[ind++] = conf->app_nrf_conf.speed;
	buffer[ind++] = conf->app_nrf_conf.power;
	buffer[ind++] = conf->app_nrf_conf.crc_type;
	buffer[ind++] = conf->app_nrf_conf.retry_delay;
	buffer[ind++] = (uint8_t)conf->app_nrf_conf.retries;
	buffer[ind++] = (uint8_t)conf->app_nrf_conf.channel;
	buffer[ind++] = (uint8_t)conf->app_nrf_conf.address[0];
	buffer[ind++] = (uint8_t)conf->app_nrf_conf.address[1];
	buffer[ind++] = (uint8_t)conf->app_nrf_conf.address[2];
	buffer[ind++] = conf->app_nrf_conf.send_crc_ack;
	buffer[ind++] = conf->app_balance_conf.pid_mode;
	buffer_append_float32_auto(buffer, conf->app_balance_conf.kp, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.ki, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.kd, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.kp2, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.ki2, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.kd2, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.hertz, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.loop_time_filter, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.fault_pitch, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.fault_roll, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.fault_duty, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.fault_adc1, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.fault_adc2, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.fault_delay_pitch, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.fault_delay_roll, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.fault_delay_duty, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.fault_delay_switch_half, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.fault_delay_switch_full, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.fault_adc_half_erpm, &ind);
	buffer[ind++] = conf->app_balance_conf.fault_is_dual_switch;
	buffer_append_float16(buffer, conf->app_balance_conf.tiltback_duty_angle, 100, &ind);
	buffer_append_float16(buffer, conf->app_balance_conf.tiltback_duty_speed, 100, &ind);
	buffer_append_float16(buffer, conf->app_balance_conf.tiltback_duty, 1000, &ind);
	buffer_append_float16(buffer, conf->app_balance_conf.tiltback_hv_angle, 100, &ind);
	buffer_append_float16(buffer, conf->app_balance_conf.tiltback_hv_speed, 100, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.tiltback_hv, &ind);
	buffer_append_float16(buffer, conf->app_balance_conf.tiltback_lv_angle, 100, &ind);
	buffer_append_float16(buffer, conf->app_balance_conf.tiltback_lv_speed, 100, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.tiltback_lv, &ind);
	buffer_append_float16(buffer, conf->app_balance_conf.tiltback_return_speed, 100, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.tiltback_constant, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.tiltback_constant_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.tiltback_variable, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.tiltback_variable_max, &ind);
	buffer_append_float16(buffer, conf->app_balance_conf.noseangling_speed, 100, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.startup_pitch_tolerance, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.startup_roll_tolerance, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.startup_speed, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.deadzone, &ind);
	buffer[ind++] = conf->app_balance_conf.multi_esc;
	buffer_append_float32_auto(buffer, conf->app_balance_conf.yaw_kp, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.yaw_ki, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.yaw_kd, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.roll_steer_kp, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.roll_steer_erpm_kp, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.brake_current, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.brake_timeout, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.yaw_current_clamp, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.ki_limit, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.kd_pt1_lowpass_frequency, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.kd_pt1_highpass_frequency, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.booster_angle, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.booster_ramp, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.booster_current, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.torquetilt_start_current, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.torquetilt_angle_limit, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.torquetilt_on_speed, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.torquetilt_off_speed, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.torquetilt_strength, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.torquetilt_filter, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.turntilt_strength, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.turntilt_angle_limit, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.turntilt_start_angle, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.turntilt_start_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->app_balance_conf.turntilt_speed, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.turntilt_erpm_boost, &ind);
	buffer_append_uint16(buffer, conf->app_balance_conf.turntilt_erpm_boost_end, &ind);
	buffer[ind++] = conf->app_pas_conf.ctrl_type;
	buffer[ind++] = conf->app_pas_conf.sensor_type;
	buffer_append_float16(buffer, conf->app_pas_conf.current_scaling, 1000, &ind);
	buffer_append_float16(buffer, conf->app_pas_conf.pedal_rpm_start, 10, &ind);
	buffer_append_float16(buffer, conf->app_pas_conf.pedal_rpm_end, 10, &ind);
	buffer[ind++] = conf->app_pas_conf.invert_pedal_direction;
	buffer_append_uint16(buffer, conf->app_pas_conf.magnets, &ind);
	buffer[ind++] = conf->app_pas_conf.use_filter;
	buffer_append_float16(buffer, conf->app_pas_conf.ramp_time_pos, 100, &ind);
	buffer_append_float16(buffer, conf->app_pas_conf.ramp_time_neg, 100, &ind);
	buffer_append_uint16(buffer, conf->app_pas_conf.update_rate_hz, &ind);
	buffer[ind++] = conf->imu_conf.type;
	buffer[ind++] = conf->imu_conf.mode;
	buffer[ind++] = conf->imu_conf.filter;
	buffer_append_float16(buffer, conf->imu_conf.accel_lowpass_filter_x, 1, &ind);
	buffer_append_float16(buffer, conf->imu_conf.accel_lowpass_filter_y, 1, &ind);
	buffer_append_float16(buffer, conf->imu_conf.accel_lowpass_filter_z, 1, &ind);
	buffer_append_float16(buffer, conf->imu_conf.gyro_lowpass_filter, 1, &ind);
	buffer_append_uint16(buffer, conf->imu_conf.sample_rate_hz, &ind);
	buffer[ind++] = conf->imu_conf.use_magnetometer;
	buffer_append_float32_auto(buffer, conf->imu_conf.accel_confidence_decay, &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.mahony_kp, &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.mahony_ki, &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.madgwick_beta, &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.rot_roll, &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.rot_pitch, &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.rot_yaw, &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.accel_offsets[0], &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.accel_offsets[1], &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.accel_offsets[2], &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.gyro_offsets[0], &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.gyro_offsets[1], &ind);
	buffer_append_float32_auto(buffer, conf->imu_conf.gyro_offsets[2], &ind);

	return ind;
}

bool VescUart::confgeneratorDeserializeMcconf(const uint8_t *buffer, mc_configuration *conf) {
	int32_t ind = 0;

	uint32_t signature = buffer_get_uint32(buffer, &ind);
	if (signature != MCCONF_SIGNATURE) {
		return false;
	}

	conf->pwm_mode = (mc_pwm_mode)buffer[ind++];
	conf->comm_mode = (mc_comm_mode)buffer[ind++];
	conf->motor_type = (mc_motor_type)buffer[ind++];
	conf->sensor_mode = (mc_sensor_mode)buffer[ind++];
	conf->l_current_max = buffer_get_float32_auto(buffer, &ind);
	conf->l_current_min = buffer_get_float32_auto(buffer, &ind);
	conf->l_in_current_max = buffer_get_float32_auto(buffer, &ind);
	conf->l_in_current_min = buffer_get_float32_auto(buffer, &ind);
	conf->l_abs_current_max = buffer_get_float32_auto(buffer, &ind);
	conf->l_min_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->l_max_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->l_erpm_start = buffer_get_float16(buffer, 10000, &ind);
	conf->l_max_erpm_fbrake = buffer_get_float32_auto(buffer, &ind);
	conf->l_max_erpm_fbrake_cc = buffer_get_float32_auto(buffer, &ind);
	conf->l_min_vin = buffer_get_float32_auto(buffer, &ind);
	conf->l_max_vin = buffer_get_float32_auto(buffer, &ind);
	conf->l_battery_cut_start = buffer_get_float32_auto(buffer, &ind);
	conf->l_battery_cut_end = buffer_get_float32_auto(buffer, &ind);
	conf->l_slow_abs_current = buffer[ind++];
	conf->l_temp_fet_start = buffer_get_float16(buffer, 10, &ind);
	conf->l_temp_fet_end = buffer_get_float16(buffer, 10, &ind);
	conf->l_temp_motor_start = buffer_get_float16(buffer, 10, &ind);
	conf->l_temp_motor_end = buffer_get_float16(buffer, 10, &ind);
	conf->l_temp_accel_dec = buffer_get_float16(buffer, 10000, &ind);
	conf->l_min_duty = buffer_get_float16(buffer, 10000, &ind);
	conf->l_max_duty = buffer_get_float16(buffer, 10000, &ind);
	conf->l_watt_max = buffer_get_float32_auto(buffer, &ind);
	conf->l_watt_min = buffer_get_float32_auto(buffer, &ind);
	conf->l_current_max_scale = buffer_get_float16(buffer, 10000, &ind);
	conf->l_current_min_scale = buffer_get_float16(buffer, 10000, &ind);
	conf->l_duty_start = buffer_get_float16(buffer, 10000, &ind);
	conf->sl_min_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->sl_min_erpm_cycle_int_limit = buffer_get_float32_auto(buffer, &ind);
	conf->sl_max_fullbreak_current_dir_change = buffer_get_float32_auto(buffer, &ind);
	conf->sl_cycle_int_limit = buffer_get_float16(buffer, 10, &ind);
	conf->sl_phase_advance_at_br = buffer_get_float16(buffer, 10000, &ind);
	conf->sl_cycle_int_rpm_br = buffer_get_float32_auto(buffer, &ind);
	conf->sl_bemf_coupling_k = buffer_get_float32_auto(buffer, &ind);
	conf->hall_table[0] = (int8_t)buffer[ind++];
	conf->hall_table[1] = (int8_t)buffer[ind++];
	conf->hall_table[2] = (int8_t)buffer[ind++];
	conf->hall_table[3] = (int8_t)buffer[ind++];
	conf->hall_table[4] = (int8_t)buffer[ind++];
	conf->hall_table[5] = (int8_t)buffer[ind++];
	conf->hall_table[6] = (int8_t)buffer[ind++];
	conf->hall_table[7] = (int8_t)buffer[ind++];
	conf->hall_sl_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->foc_current_kp = buffer_get_float32_auto(buffer, &ind);
	conf->foc_current_ki = buffer_get_float32_auto(buffer, &ind);
	conf->foc_f_zv = buffer_get_float32_auto(buffer, &ind);
	conf->foc_dt_us = buffer_get_float32_auto(buffer, &ind);
	conf->foc_encoder_inverted = buffer[ind++];
	conf->foc_encoder_offset = buffer_get_float32_auto(buffer, &ind);
	conf->foc_encoder_ratio = buffer_get_float32_auto(buffer, &ind);
	conf->foc_sensor_mode = (mc_foc_sensor_mode)buffer[ind++];
	conf->foc_pll_kp = buffer_get_float32_auto(buffer, &ind);
	conf->foc_pll_ki = buffer_get_float32_auto(buffer, &ind);
	conf->foc_motor_l = buffer_get_float32_auto(buffer, &ind);
	conf->foc_motor_ld_lq_diff = buffer_get_float32_auto(buffer, &ind);
	conf->foc_motor_r = buffer_get_float32_auto(buffer, &ind);
	conf->foc_motor_flux_linkage = buffer_get_float32_auto(buffer, &ind);
	conf->foc_observer_gain = buffer_get_float32_auto(buffer, &ind);
	conf->foc_observer_gain_slow = buffer_get_float32_auto(buffer, &ind);
	conf->foc_observer_offset = buffer_get_float16(buffer, 1000, &ind);
	conf->foc_duty_dowmramp_kp = buffer_get_float32_auto(buffer, &ind);
	conf->foc_duty_dowmramp_ki = buffer_get_float32_auto(buffer, &ind);
	conf->foc_start_curr_dec = buffer_get_float16(buffer, 10000, &ind);
	conf->foc_start_curr_dec_rpm = buffer_get_float32_auto(buffer, &ind);
	conf->foc_openloop_rpm = buffer_get_float32_auto(buffer, &ind);
	conf->foc_openloop_rpm_low = buffer_get_float16(buffer, 1000, &ind);
	conf->foc_d_gain_scale_start = buffer_get_float16(buffer, 1000, &ind);
	conf->foc_d_gain_scale_max_mod = buffer_get_float16(buffer, 1000, &ind);
	conf->foc_sl_openloop_hyst = buffer_get_float16(buffer, 100, &ind);
	conf->foc_sl_openloop_time_lock = buffer_get_float16(buffer, 100, &ind);
	conf->foc_sl_openloop_time_ramp = buffer_get_float16(buffer, 100, &ind);
	conf->foc_sl_openloop_time = buffer_get_float16(buffer, 100, &ind);
	conf->foc_sl_openloop_boost_q = buffer_get_float16(buffer, 100, &ind);
	conf->foc_sl_openloop_max_q = buffer_get_float16(buffer, 100, &ind);
	conf->foc_hall_table[0] = buffer[ind++];
	conf->foc_hall_table[1] = buffer[ind++];
	conf->foc_hall_table[2] = buffer[ind++];
	conf->foc_hall_table[3] = buffer[ind++];
	conf->foc_hall_table[4] = buffer[ind++];
	conf->foc_hall_table[5] = buffer[ind++];
	conf->foc_hall_table[6] = buffer[ind++];
	conf->foc_hall_table[7] = buffer[ind++];
	conf->foc_hall_interp_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->foc_sl_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->foc_sample_v0_v7 = buffer[ind++];
	conf->foc_sample_high_current = buffer[ind++];
	conf->foc_sat_comp_mode = (SAT_COMP_MODE)buffer[ind++];
	conf->foc_sat_comp = buffer_get_float16(buffer, 1000, &ind);
	conf->foc_temp_comp = buffer[ind++];
	conf->foc_temp_comp_base_temp = buffer_get_float16(buffer, 100, &ind);
	conf->foc_current_filter_const = buffer_get_float16(buffer, 10000, &ind);
	conf->foc_cc_decoupling = (mc_foc_cc_decoupling_mode)buffer[ind++];
	conf->foc_observer_type = (mc_foc_observer_type)buffer[ind++];
	conf->foc_hfi_voltage_start = buffer_get_float16(buffer, 10, &ind);
	conf->foc_hfi_voltage_run = buffer_get_float16(buffer, 10, &ind);
	conf->foc_hfi_voltage_max = buffer_get_float16(buffer, 10, &ind);
	conf->foc_hfi_gain = buffer_get_float16(buffer, 1000, &ind);
	conf->foc_hfi_hyst = buffer_get_float16(buffer, 100, &ind);
	conf->foc_sl_erpm_hfi = buffer_get_float32_auto(buffer, &ind);
	conf->foc_hfi_start_samples = buffer_get_uint16(buffer, &ind);
	conf->foc_hfi_obs_ovr_sec = buffer_get_float32_auto(buffer, &ind);
	conf->foc_hfi_samples = (_foc_hfi_samples)buffer[ind++];
	conf->foc_offsets_cal_on_boot = buffer[ind++];
	conf->foc_offsets_current[0] = buffer_get_float32_auto(buffer, &ind);
	conf->foc_offsets_current[1] = buffer_get_float32_auto(buffer, &ind);
	conf->foc_offsets_current[2] = buffer_get_float32_auto(buffer, &ind);
	conf->foc_offsets_voltage[0] = buffer_get_float16(buffer, 10000, &ind);
	conf->foc_offsets_voltage[1] = buffer_get_float16(buffer, 10000, &ind);
	conf->foc_offsets_voltage[2] = buffer_get_float16(buffer, 10000, &ind);
	conf->foc_offsets_voltage_undriven[0] = buffer_get_float16(buffer, 10000, &ind);
	conf->foc_offsets_voltage_undriven[1] = buffer_get_float16(buffer, 10000, &ind);
	conf->foc_offsets_voltage_undriven[2] = buffer_get_float16(buffer, 10000, &ind);
	conf->foc_phase_filter_enable = buffer[ind++];
	conf->foc_phase_filter_disable_fault = buffer[ind++];
	conf->foc_phase_filter_max_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->foc_mtpa_mode = (MTPA_MODE)buffer[ind++];
	conf->foc_fw_current_max = buffer_get_float32_auto(buffer, &ind);
	conf->foc_fw_duty_start = buffer_get_float16(buffer, 10000, &ind);
	conf->foc_fw_ramp_time = buffer_get_float16(buffer, 1000, &ind);
	conf->foc_fw_q_current_factor = buffer_get_float16(buffer, 10000, &ind);
	conf->foc_speed_soure = (SPEED_SRC)buffer[ind++];
	conf->gpd_buffer_notify_left = buffer_get_int16(buffer, &ind);
	conf->gpd_buffer_interpol = buffer_get_int16(buffer, &ind);
	conf->gpd_current_filter_const = buffer_get_float16(buffer, 10000, &ind);
	conf->gpd_current_kp = buffer_get_float32_auto(buffer, &ind);
	conf->gpd_current_ki = buffer_get_float32_auto(buffer, &ind);
	conf->sp_pid_loop_rate = (PID_RATE)buffer[ind++];
	conf->s_pid_kp = buffer_get_float32_auto(buffer, &ind);
	conf->s_pid_ki = buffer_get_float32_auto(buffer, &ind);
	conf->s_pid_kd = buffer_get_float32_auto(buffer, &ind);
	conf->s_pid_kd_filter = buffer_get_float16(buffer, 10000, &ind);
	conf->s_pid_min_erpm = buffer_get_float32_auto(buffer, &ind);
	conf->s_pid_allow_braking = buffer[ind++];
	conf->s_pid_ramp_erpms_s = buffer_get_float32_auto(buffer, &ind);
	conf->p_pid_kp = buffer_get_float32_auto(buffer, &ind);
	conf->p_pid_ki = buffer_get_float32_auto(buffer, &ind);
	conf->p_pid_kd = buffer_get_float32_auto(buffer, &ind);
	conf->p_pid_kd_proc = buffer_get_float32_auto(buffer, &ind);
	conf->p_pid_kd_filter = buffer_get_float16(buffer, 10000, &ind);
	conf->p_pid_ang_div = buffer_get_float32_auto(buffer, &ind);
	conf->p_pid_gain_dec_angle = buffer_get_float16(buffer, 10, &ind);
	conf->p_pid_offset = buffer_get_float32_auto(buffer, &ind);
	conf->cc_startup_boost_duty = buffer_get_float16(buffer, 10000, &ind);
	conf->cc_min_current = buffer_get_float32_auto(buffer, &ind);
	conf->cc_gain = buffer_get_float32_auto(buffer, &ind);
	conf->cc_ramp_step_max = buffer_get_float16(buffer, 10000, &ind);
	conf->m_fault_stop_time_ms = buffer_get_int32(buffer, &ind);
	conf->m_duty_ramp_step = buffer_get_float16(buffer, 10000, &ind);
	conf->m_current_backoff_gain = buffer_get_float32_auto(buffer, &ind);
	conf->m_encoder_counts = buffer_get_uint32(buffer, &ind);
	conf->m_encoder_sin_amp = buffer_get_float16(buffer, 1000, &ind);
	conf->m_encoder_cos_amp = buffer_get_float16(buffer, 1000, &ind);
	conf->m_encoder_sin_offset = buffer_get_float16(buffer, 1000, &ind);
	conf->m_encoder_cos_offset = buffer_get_float16(buffer, 1000, &ind);
	conf->m_encoder_sincos_filter_constant = buffer_get_float16(buffer, 1000, &ind);
	conf->m_encoder_sincos_phase_correction = buffer_get_float16(buffer, 1000, &ind);
	conf->m_sensor_port_mode = (sensor_port_mode)buffer[ind++];
	conf->m_invert_direction = buffer[ind++];
	conf->m_drv8301_oc_mode =(drv8301_oc_mode)buffer[ind++];
	conf->m_drv8301_oc_adj = buffer[ind++];
	conf->m_bldc_f_sw_min = buffer_get_float32_auto(buffer, &ind);
	conf->m_bldc_f_sw_max = buffer_get_float32_auto(buffer, &ind);
	conf->m_dc_f_sw = buffer_get_float32_auto(buffer, &ind);
	conf->m_ntc_motor_beta = buffer_get_float32_auto(buffer, &ind);
	conf->m_out_aux_mode = (out_aux_mode)buffer[ind++];
	conf->m_motor_temp_sens_type = (temp_sensor_type)buffer[ind++];
	conf->m_ptc_motor_coeff = buffer_get_float32_auto(buffer, &ind);
	conf->m_ntcx_ptcx_res = buffer_get_float16(buffer, 0.1, &ind);
	conf->m_ntcx_ptcx_temp_base = buffer_get_float16(buffer, 10, &ind);
	conf->m_hall_extra_samples = buffer[ind++];
	conf->m_batt_filter_const = buffer[ind++];
	conf->si_motor_poles = buffer[ind++];
	conf->si_gear_ratio = buffer_get_float32_auto(buffer, &ind);
	conf->si_wheel_diameter = buffer_get_float32_auto(buffer, &ind);
	conf->si_battery_type = (BATTERY_TYPE)buffer[ind++];
	conf->si_battery_cells = buffer[ind++];
	conf->si_battery_ah = buffer_get_float32_auto(buffer, &ind);
	conf->si_motor_nl_current = buffer_get_float32_auto(buffer, &ind);
	conf->bms.type = (BMS_TYPE)buffer[ind++];
	conf->bms.limit_mode = buffer[ind++];
	conf->bms.t_limit_start = buffer_get_float16(buffer, 100, &ind);
	conf->bms.t_limit_end = buffer_get_float16(buffer, 100, &ind);
	conf->bms.soc_limit_start = buffer_get_float16(buffer, 1000, &ind);
	conf->bms.soc_limit_end = buffer_get_float16(buffer, 1000, &ind);
	conf->bms.fwd_can_mode =(BMS_FWD_CAN_MODE)buffer[ind++];

	return true;
}

int32_t VescUart::confgeneratorSerializeMcconf(uint8_t *buffer, const mc_configuration *conf){
	int32_t ind = 0;

	buffer_append_uint32(buffer, MCCONF_SIGNATURE, &ind);

	buffer[ind++] = conf->pwm_mode;
	buffer[ind++] = conf->comm_mode;
	buffer[ind++] = conf->motor_type;
	buffer[ind++] = conf->sensor_mode;
	buffer_append_float32_auto(buffer, conf->l_current_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_current_min, &ind);
	buffer_append_float32_auto(buffer, conf->l_in_current_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_in_current_min, &ind);
	buffer_append_float32_auto(buffer, conf->l_abs_current_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_min_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_erpm, &ind);
	buffer_append_float16(buffer, conf->l_erpm_start, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_erpm_fbrake, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_erpm_fbrake_cc, &ind);
	buffer_append_float32_auto(buffer, conf->l_min_vin, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_vin, &ind);
	buffer_append_float32_auto(buffer, conf->l_battery_cut_start, &ind);
	buffer_append_float32_auto(buffer, conf->l_battery_cut_end, &ind);
	buffer[ind++] = conf->l_slow_abs_current;
	buffer_append_float16(buffer, conf->l_temp_fet_start, 10, &ind);
	buffer_append_float16(buffer, conf->l_temp_fet_end, 10, &ind);
	buffer_append_float16(buffer, conf->l_temp_motor_start, 10, &ind);
	buffer_append_float16(buffer, conf->l_temp_motor_end, 10, &ind);
	buffer_append_float16(buffer, conf->l_temp_accel_dec, 10000, &ind);
	buffer_append_float16(buffer, conf->l_min_duty, 10000, &ind);
	buffer_append_float16(buffer, conf->l_max_duty, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->l_watt_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_watt_min, &ind);
	buffer_append_float16(buffer, conf->l_current_max_scale, 10000, &ind);
	buffer_append_float16(buffer, conf->l_current_min_scale, 10000, &ind);
	buffer_append_float16(buffer, conf->l_duty_start, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->sl_min_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->sl_min_erpm_cycle_int_limit, &ind);
	buffer_append_float32_auto(buffer, conf->sl_max_fullbreak_current_dir_change, &ind);
	buffer_append_float16(buffer, conf->sl_cycle_int_limit, 10, &ind);
	buffer_append_float16(buffer, conf->sl_phase_advance_at_br, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->sl_cycle_int_rpm_br, &ind);
	buffer_append_float32_auto(buffer, conf->sl_bemf_coupling_k, &ind);
	buffer[ind++] = (uint8_t)conf->hall_table[0];
	buffer[ind++] = (uint8_t)conf->hall_table[1];
	buffer[ind++] = (uint8_t)conf->hall_table[2];
	buffer[ind++] = (uint8_t)conf->hall_table[3];
	buffer[ind++] = (uint8_t)conf->hall_table[4];
	buffer[ind++] = (uint8_t)conf->hall_table[5];
	buffer[ind++] = (uint8_t)conf->hall_table[6];
	buffer[ind++] = (uint8_t)conf->hall_table[7];
	buffer_append_float32_auto(buffer, conf->hall_sl_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->foc_current_kp, &ind);
	buffer_append_float32_auto(buffer, conf->foc_current_ki, &ind);
	buffer_append_float32_auto(buffer, conf->foc_f_zv, &ind);
	buffer_append_float32_auto(buffer, conf->foc_dt_us, &ind);
	buffer[ind++] = conf->foc_encoder_inverted;
	buffer_append_float32_auto(buffer, conf->foc_encoder_offset, &ind);
	buffer_append_float32_auto(buffer, conf->foc_encoder_ratio, &ind);
	buffer[ind++] = conf->foc_sensor_mode;
	buffer_append_float32_auto(buffer, conf->foc_pll_kp, &ind);
	buffer_append_float32_auto(buffer, conf->foc_pll_ki, &ind);
	buffer_append_float32_auto(buffer, conf->foc_motor_l, &ind);
	buffer_append_float32_auto(buffer, conf->foc_motor_ld_lq_diff, &ind);
	buffer_append_float32_auto(buffer, conf->foc_motor_r, &ind);
	buffer_append_float32_auto(buffer, conf->foc_motor_flux_linkage, &ind);
	buffer_append_float32_auto(buffer, conf->foc_observer_gain, &ind);
	buffer_append_float32_auto(buffer, conf->foc_observer_gain_slow, &ind);
	buffer_append_float16(buffer, conf->foc_observer_offset, 1000, &ind);
	buffer_append_float32_auto(buffer, conf->foc_duty_dowmramp_kp, &ind);
	buffer_append_float32_auto(buffer, conf->foc_duty_dowmramp_ki, &ind);
	buffer_append_float16(buffer, conf->foc_start_curr_dec, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->foc_start_curr_dec_rpm, &ind);
	buffer_append_float32_auto(buffer, conf->foc_openloop_rpm, &ind);
	buffer_append_float16(buffer, conf->foc_openloop_rpm_low, 1000, &ind);
	buffer_append_float16(buffer, conf->foc_d_gain_scale_start, 1000, &ind);
	buffer_append_float16(buffer, conf->foc_d_gain_scale_max_mod, 1000, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_hyst, 100, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_time_lock, 100, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_time_ramp, 100, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_time, 100, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_boost_q, 100, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_max_q, 100, &ind);
	buffer[ind++] = (uint8_t)conf->foc_hall_table[0];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[1];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[2];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[3];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[4];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[5];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[6];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[7];
	buffer_append_float32_auto(buffer, conf->foc_hall_interp_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->foc_sl_erpm, &ind);
	buffer[ind++] = conf->foc_sample_v0_v7;
	buffer[ind++] = conf->foc_sample_high_current;
	buffer[ind++] = conf->foc_sat_comp_mode;
	buffer_append_float16(buffer, conf->foc_sat_comp, 1000, &ind);
	buffer[ind++] = conf->foc_temp_comp;
	buffer_append_float16(buffer, conf->foc_temp_comp_base_temp, 100, &ind);
	buffer_append_float16(buffer, conf->foc_current_filter_const, 10000, &ind);
	buffer[ind++] = conf->foc_cc_decoupling;
	buffer[ind++] = conf->foc_observer_type;
	buffer_append_float16(buffer, conf->foc_hfi_voltage_start, 10, &ind);
	buffer_append_float16(buffer, conf->foc_hfi_voltage_run, 10, &ind);
	buffer_append_float16(buffer, conf->foc_hfi_voltage_max, 10, &ind);
	buffer_append_float16(buffer, conf->foc_hfi_gain, 1000, &ind);
	buffer_append_float16(buffer, conf->foc_hfi_hyst, 100, &ind);
	buffer_append_float32_auto(buffer, conf->foc_sl_erpm_hfi, &ind);
	buffer_append_uint16(buffer, conf->foc_hfi_start_samples, &ind);
	buffer_append_float32_auto(buffer, conf->foc_hfi_obs_ovr_sec, &ind);
	buffer[ind++] = conf->foc_hfi_samples;
	buffer[ind++] = conf->foc_offsets_cal_on_boot;
	buffer_append_float32_auto(buffer, conf->foc_offsets_current[0], &ind);
	buffer_append_float32_auto(buffer, conf->foc_offsets_current[1], &ind);
	buffer_append_float32_auto(buffer, conf->foc_offsets_current[2], &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage[0], 10000, &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage[1], 10000, &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage[2], 10000, &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage_undriven[0], 10000, &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage_undriven[1], 10000, &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage_undriven[2], 10000, &ind);
	buffer[ind++] = conf->foc_phase_filter_enable;
	buffer[ind++] = conf->foc_phase_filter_disable_fault;
	buffer_append_float32_auto(buffer, conf->foc_phase_filter_max_erpm, &ind);
	buffer[ind++] = conf->foc_mtpa_mode;
	buffer_append_float32_auto(buffer, conf->foc_fw_current_max, &ind);
	buffer_append_float16(buffer, conf->foc_fw_duty_start, 10000, &ind);
	buffer_append_float16(buffer, conf->foc_fw_ramp_time, 1000, &ind);
	buffer_append_float16(buffer, conf->foc_fw_q_current_factor, 10000, &ind);
	buffer[ind++] = conf->foc_speed_soure;
	buffer_append_int16(buffer, conf->gpd_buffer_notify_left, &ind);
	buffer_append_int16(buffer, conf->gpd_buffer_interpol, &ind);
	buffer_append_float16(buffer, conf->gpd_current_filter_const, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->gpd_current_kp, &ind);
	buffer_append_float32_auto(buffer, conf->gpd_current_ki, &ind);
	buffer[ind++] = conf->sp_pid_loop_rate;
	buffer_append_float32_auto(buffer, conf->s_pid_kp, &ind);
	buffer_append_float32_auto(buffer, conf->s_pid_ki, &ind);
	buffer_append_float32_auto(buffer, conf->s_pid_kd, &ind);
	buffer_append_float16(buffer, conf->s_pid_kd_filter, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->s_pid_min_erpm, &ind);
	buffer[ind++] = conf->s_pid_allow_braking;
	buffer_append_float32_auto(buffer, conf->s_pid_ramp_erpms_s, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_kp, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_ki, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_kd, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_kd_proc, &ind);
	buffer_append_float16(buffer, conf->p_pid_kd_filter, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_ang_div, &ind);
	buffer_append_float16(buffer, conf->p_pid_gain_dec_angle, 10, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_offset, &ind);
	buffer_append_float16(buffer, conf->cc_startup_boost_duty, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->cc_min_current, &ind);
	buffer_append_float32_auto(buffer, conf->cc_gain, &ind);
	buffer_append_float16(buffer, conf->cc_ramp_step_max, 10000, &ind);
	buffer_append_int32(buffer, conf->m_fault_stop_time_ms, &ind);
	buffer_append_float16(buffer, conf->m_duty_ramp_step, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->m_current_backoff_gain, &ind);
	buffer_append_uint32(buffer, conf->m_encoder_counts, &ind);
	buffer_append_float16(buffer, conf->m_encoder_sin_amp, 1000, &ind);
	buffer_append_float16(buffer, conf->m_encoder_cos_amp, 1000, &ind);
	buffer_append_float16(buffer, conf->m_encoder_sin_offset, 1000, &ind);
	buffer_append_float16(buffer, conf->m_encoder_cos_offset, 1000, &ind);
	buffer_append_float16(buffer, conf->m_encoder_sincos_filter_constant, 1000, &ind);
	buffer_append_float16(buffer, conf->m_encoder_sincos_phase_correction, 1000, &ind);
	buffer[ind++] = conf->m_sensor_port_mode;
	buffer[ind++] = conf->m_invert_direction;
	buffer[ind++] = conf->m_drv8301_oc_mode;
	buffer[ind++] = (uint8_t)conf->m_drv8301_oc_adj;
	buffer_append_float32_auto(buffer, conf->m_bldc_f_sw_min, &ind);
	buffer_append_float32_auto(buffer, conf->m_bldc_f_sw_max, &ind);
	buffer_append_float32_auto(buffer, conf->m_dc_f_sw, &ind);
	buffer_append_float32_auto(buffer, conf->m_ntc_motor_beta, &ind);
	buffer[ind++] = conf->m_out_aux_mode;
	buffer[ind++] = conf->m_motor_temp_sens_type;
	buffer_append_float32_auto(buffer, conf->m_ptc_motor_coeff, &ind);
	buffer_append_float16(buffer, conf->m_ntcx_ptcx_res, 0.1, &ind);
	buffer_append_float16(buffer, conf->m_ntcx_ptcx_temp_base, 10, &ind);
	buffer[ind++] = (uint8_t)conf->m_hall_extra_samples;
	buffer[ind++] = (uint8_t)conf->m_batt_filter_const;
	buffer[ind++] = (uint8_t)conf->si_motor_poles;
	buffer_append_float32_auto(buffer, conf->si_gear_ratio, &ind);
	buffer_append_float32_auto(buffer, conf->si_wheel_diameter, &ind);
	buffer[ind++] = conf->si_battery_type;
	buffer[ind++] = (uint8_t)conf->si_battery_cells;
	buffer_append_float32_auto(buffer, conf->si_battery_ah, &ind);
	buffer_append_float32_auto(buffer, conf->si_motor_nl_current, &ind);
	buffer[ind++] = conf->bms.type;
	buffer[ind++] = conf->bms.limit_mode;
	buffer_append_float16(buffer, conf->bms.t_limit_start, 100, &ind);
	buffer_append_float16(buffer, conf->bms.t_limit_end, 100, &ind);
	buffer_append_float16(buffer, conf->bms.soc_limit_start, 1000, &ind);
	buffer_append_float16(buffer, conf->bms.soc_limit_end, 1000, &ind);
	buffer[ind++] = conf->bms.fwd_can_mode;

	return ind;
}

void VescUart::printVescValues() {
	if(debugPort != NULL){
		debugPort->print("avgMotorCurrent: "); 	debugPort->println(data.avgMotorCurrent);
		debugPort->print("avgInputCurrent: "); 	debugPort->println(data.avgInputCurrent);
		debugPort->print("dutyCycleNow: "); 	debugPort->println(data.dutyCycleNow);
		debugPort->print("rpm: "); 				debugPort->println(data.rpm);
		debugPort->print("inputVoltage: "); 	debugPort->println(data.inpVoltage);
		debugPort->print("ampHours: "); 		debugPort->println(data.ampHours);
		debugPort->print("ampHoursCharged: "); 	debugPort->println(data.ampHoursCharged);
		debugPort->print("wattHours: "); 		debugPort->println(data.wattHours);
		debugPort->print("wattHoursCharged: "); debugPort->println(data.wattHoursCharged);
		debugPort->print("tachometer: "); 		debugPort->println(data.tachometer);
		debugPort->print("tachometerAbs: "); 	debugPort->println(data.tachometerAbs);
		debugPort->print("tempMosfet: "); 		debugPort->println(data.tempMosfet);
		debugPort->print("tempMotor: "); 		debugPort->println(data.tempMotor);
		debugPort->print("error: "); 			debugPort->println(data.error);
	}
}