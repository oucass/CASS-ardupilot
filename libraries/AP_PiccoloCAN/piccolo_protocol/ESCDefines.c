// ESCDefines.c was generated by ProtoGen version 2.18.c

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 

#include "ESCDefines.h"
#include "fielddecode.h"
#include "fieldencode.h"
#include "scaleddecode.h"
#include "scaledencode.h"

/*!
 * \brief Encode a ESC_StatusBits_t structure into a byte array
 *
 * The *status* of the ESC is represented using these status bits. ESC system
 * functionality can be quickly determined using these bits
 * \param _pg_data points to the byte array to add encoded data to
 * \param _pg_bytecount points to the starting location in the byte array, and will be incremented by the number of encoded bytes.
 * \param _pg_user is the data to encode in the byte array
 */
void encodeESC_StatusBits_t(uint8_t* _pg_data, int* _pg_bytecount, const ESC_StatusBits_t* _pg_user)
{
    int _pg_byteindex = *_pg_bytecount;

    // 1 = Hardware inhibit is active (ESC is disabled)
    _pg_data[_pg_byteindex] = (uint8_t)_pg_user->hwInhibit << 7;

    // 1 = Software inhibit is active (ESC is disabled)
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->swInhibit << 6;

    // 0 = Active Freewheeling is not enabled, 1 = Active Freewheeling is enabled
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->afwEnabled << 5;

    // 0 = Motor direction is FORWARDS, 1= Motor direction is REVERSE
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->direction << 4;

    // Set if the ESC command timeout period has elapsed (and the ESC is in STANDBY mode)
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->timeout << 3;

    // 1 = in starting mode (0 = stopped or running)
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->starting << 2;

    // 0 = most recent command from CAN, 1 = most recent command from PWM
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->commandSource << 1;

    // ESC is running
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->running;
    _pg_byteindex += 1; // close bit field

    *_pg_bytecount = _pg_byteindex;

}// encodeESC_StatusBits_t

/*!
 * \brief Decode a ESC_StatusBits_t structure from a byte array
 *
 * The *status* of the ESC is represented using these status bits. ESC system
 * functionality can be quickly determined using these bits
 * \param _pg_data points to the byte array to decoded data from
 * \param _pg_bytecount points to the starting location in the byte array, and will be incremented by the number of bytes decoded
 * \param _pg_user is the data to decode from the byte array
 * \return 1 if the data are decoded, else 0. If 0 is returned _pg_bytecount will not be updated.
 */
int decodeESC_StatusBits_t(const uint8_t* _pg_data, int* _pg_bytecount, ESC_StatusBits_t* _pg_user)
{
    int _pg_byteindex = *_pg_bytecount;

    // 1 = Hardware inhibit is active (ESC is disabled)
    _pg_user->hwInhibit = (_pg_data[_pg_byteindex] >> 7);

    // 1 = Software inhibit is active (ESC is disabled)
    _pg_user->swInhibit = ((_pg_data[_pg_byteindex] >> 6) & 0x1);

    // 0 = Active Freewheeling is not enabled, 1 = Active Freewheeling is enabled
    _pg_user->afwEnabled = ((_pg_data[_pg_byteindex] >> 5) & 0x1);

    // 0 = Motor direction is FORWARDS, 1= Motor direction is REVERSE
    _pg_user->direction = ((_pg_data[_pg_byteindex] >> 4) & 0x1);

    // Set if the ESC command timeout period has elapsed (and the ESC is in STANDBY mode)
    _pg_user->timeout = ((_pg_data[_pg_byteindex] >> 3) & 0x1);

    // 1 = in starting mode (0 = stopped or running)
    _pg_user->starting = ((_pg_data[_pg_byteindex] >> 2) & 0x1);

    // 0 = most recent command from CAN, 1 = most recent command from PWM
    _pg_user->commandSource = ((_pg_data[_pg_byteindex] >> 1) & 0x1);

    // ESC is running
    _pg_user->running = ((_pg_data[_pg_byteindex]) & 0x1);
    _pg_byteindex += 1; // close bit field

    *_pg_bytecount = _pg_byteindex;

    return 1;

}// decodeESC_StatusBits_t

/*!
 * \brief Encode a ESC_WarningBits_t structure into a byte array
 *
 * The *warning* bits enumerate various system warnings/errors of which the user
 * (or user software) should be made aware. These *warning* bits are transmitted
 * in the telemetry packets such that user software is aware of any these
 * *warning* conditions and can poll the ESC for particular packets if any
 * further information is needed. The ESC will continue to function in the case
 * of a *warning* state
 * \param _pg_data points to the byte array to add encoded data to
 * \param _pg_bytecount points to the starting location in the byte array, and will be incremented by the number of encoded bytes.
 * \param _pg_user is the data to encode in the byte array
 */
void encodeESC_WarningBits_t(uint8_t* _pg_data, int* _pg_bytecount, const ESC_WarningBits_t* _pg_user)
{
    int _pg_byteindex = *_pg_bytecount;

    // Set if RPM signal is not detected
    _pg_data[_pg_byteindex] = (uint8_t)_pg_user->noRPMSignal << 7;

    // Set if the ESC motor speed exceeds the configured warning threshold
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->overspeed << 6;

    // Set if the ESC motor current (positive or negative) exceeds the configured warning threshold
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->overcurrent << 5;

    // Set if the internal ESC temperature is above the warning threshold
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->escTemperature << 4;

    // Set if the motor temperature is above the warning threshold
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->motorTemperature << 3;

    // Set if the input voltage is below the minimum threshold
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->undervoltage << 2;

    // Set if the input voltage is above the maximum threshold
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->overvoltage << 1;

    // Set if hardware PWM input is enabled but invalid
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->invalidPWMsignal;
    _pg_byteindex += 1; // close bit field

    *_pg_bytecount = _pg_byteindex;

}// encodeESC_WarningBits_t

/*!
 * \brief Decode a ESC_WarningBits_t structure from a byte array
 *
 * The *warning* bits enumerate various system warnings/errors of which the user
 * (or user software) should be made aware. These *warning* bits are transmitted
 * in the telemetry packets such that user software is aware of any these
 * *warning* conditions and can poll the ESC for particular packets if any
 * further information is needed. The ESC will continue to function in the case
 * of a *warning* state
 * \param _pg_data points to the byte array to decoded data from
 * \param _pg_bytecount points to the starting location in the byte array, and will be incremented by the number of bytes decoded
 * \param _pg_user is the data to decode from the byte array
 * \return 1 if the data are decoded, else 0. If 0 is returned _pg_bytecount will not be updated.
 */
int decodeESC_WarningBits_t(const uint8_t* _pg_data, int* _pg_bytecount, ESC_WarningBits_t* _pg_user)
{
    int _pg_byteindex = *_pg_bytecount;

    // Set if RPM signal is not detected
    _pg_user->noRPMSignal = (_pg_data[_pg_byteindex] >> 7);

    // Set if the ESC motor speed exceeds the configured warning threshold
    _pg_user->overspeed = ((_pg_data[_pg_byteindex] >> 6) & 0x1);

    // Set if the ESC motor current (positive or negative) exceeds the configured warning threshold
    _pg_user->overcurrent = ((_pg_data[_pg_byteindex] >> 5) & 0x1);

    // Set if the internal ESC temperature is above the warning threshold
    _pg_user->escTemperature = ((_pg_data[_pg_byteindex] >> 4) & 0x1);

    // Set if the motor temperature is above the warning threshold
    _pg_user->motorTemperature = ((_pg_data[_pg_byteindex] >> 3) & 0x1);

    // Set if the input voltage is below the minimum threshold
    _pg_user->undervoltage = ((_pg_data[_pg_byteindex] >> 2) & 0x1);

    // Set if the input voltage is above the maximum threshold
    _pg_user->overvoltage = ((_pg_data[_pg_byteindex] >> 1) & 0x1);

    // Set if hardware PWM input is enabled but invalid
    _pg_user->invalidPWMsignal = ((_pg_data[_pg_byteindex]) & 0x1);
    _pg_byteindex += 1; // close bit field

    *_pg_bytecount = _pg_byteindex;

    return 1;

}// decodeESC_WarningBits_t

/*!
 * \brief Encode a ESC_ErrorBits_t structure into a byte array
 *
 * The *error* bits enumerate critical system errors that will cause the ESC to
 * stop functioning until the error cases are alleviated
 * \param _pg_data points to the byte array to add encoded data to
 * \param _pg_bytecount points to the starting location in the byte array, and will be incremented by the number of encoded bytes.
 * \param _pg_user is the data to encode in the byte array
 */
void encodeESC_ErrorBits_t(uint8_t* _pg_data, int* _pg_bytecount, const ESC_ErrorBits_t* _pg_user)
{
    int _pg_byteindex = *_pg_bytecount;

    // Set if communication link to the motor controller is lost
    _pg_data[_pg_byteindex] = (uint8_t)_pg_user->linkError << 7;

    // Set if the ESC has detected an overcurrent event and is actively folding back duty cycle
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->foldback << 6;

    // Set if the settings checksum does not match the programmed values
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->settingsChecksum << 5;

    // Set if the motor settings are invalid
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->motorSettings << 4;

    // Reserved for future use
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->reservedD << 3;

    // Reserved for future use
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->reservedE << 2;

    // Reserved for future use
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->reservedF << 1;

    // Reserved for future use
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->reservedG;
    _pg_byteindex += 1; // close bit field

    *_pg_bytecount = _pg_byteindex;

}// encodeESC_ErrorBits_t

/*!
 * \brief Decode a ESC_ErrorBits_t structure from a byte array
 *
 * The *error* bits enumerate critical system errors that will cause the ESC to
 * stop functioning until the error cases are alleviated
 * \param _pg_data points to the byte array to decoded data from
 * \param _pg_bytecount points to the starting location in the byte array, and will be incremented by the number of bytes decoded
 * \param _pg_user is the data to decode from the byte array
 * \return 1 if the data are decoded, else 0. If 0 is returned _pg_bytecount will not be updated.
 */
int decodeESC_ErrorBits_t(const uint8_t* _pg_data, int* _pg_bytecount, ESC_ErrorBits_t* _pg_user)
{
    int _pg_byteindex = *_pg_bytecount;

    // Set if communication link to the motor controller is lost
    _pg_user->linkError = (_pg_data[_pg_byteindex] >> 7);

    // Set if the ESC has detected an overcurrent event and is actively folding back duty cycle
    _pg_user->foldback = ((_pg_data[_pg_byteindex] >> 6) & 0x1);

    // Set if the settings checksum does not match the programmed values
    _pg_user->settingsChecksum = ((_pg_data[_pg_byteindex] >> 5) & 0x1);

    // Set if the motor settings are invalid
    _pg_user->motorSettings = ((_pg_data[_pg_byteindex] >> 4) & 0x1);

    // Reserved for future use
    _pg_user->reservedD = ((_pg_data[_pg_byteindex] >> 3) & 0x1);

    // Reserved for future use
    _pg_user->reservedE = ((_pg_data[_pg_byteindex] >> 2) & 0x1);

    // Reserved for future use
    _pg_user->reservedF = ((_pg_data[_pg_byteindex] >> 1) & 0x1);

    // Reserved for future use
    _pg_user->reservedG = ((_pg_data[_pg_byteindex]) & 0x1);
    _pg_byteindex += 1; // close bit field

    *_pg_bytecount = _pg_byteindex;

    return 1;

}// decodeESC_ErrorBits_t

/*!
 * \brief Encode a ESC_TelemetryPackets_t structure into a byte array
 *
 * These bits are used to determine which packets are automatically transmitted
 * as telemetry data by the ESC. Only the packets described here can be
 * configured as telemetry packets
 * \param _pg_data points to the byte array to add encoded data to
 * \param _pg_bytecount points to the starting location in the byte array, and will be incremented by the number of encoded bytes.
 * \param _pg_user is the data to encode in the byte array
 */
void encodeESC_TelemetryPackets_t(uint8_t* _pg_data, int* _pg_bytecount, const ESC_TelemetryPackets_t* _pg_user)
{
    int _pg_byteindex = *_pg_bytecount;

    // If this bit is set, the STATUS_A packet will be transmitted at the configured rate
    _pg_data[_pg_byteindex] = (uint8_t)_pg_user->statusA << 7;

    // If this bit is set, the STATUS_B packet will be transmitted at the configured rate
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->statusB << 6;

    // If this bit is set, the STATUS_C packet will be transmitted at the configured rate
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->statusC << 5;

    // If this bit is set, the ACCELEROMETER packet will be transmitted at the configured rate
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->accelerometer << 4;

    // If this bit is set, the STATUS_D packet will be transmitted at the configured rate
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->statusD << 3;

    // Reserved for future use
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->reservedB << 2;

    // If this bit is set, any STATUS_x packets selected for telemetry will be mirrored on the Piccolo Downlink packet group (0x14)
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->piccoloDownlink << 1;

    // Reserved for future use
    _pg_data[_pg_byteindex] |= (uint8_t)_pg_user->reservedD;
    _pg_byteindex += 1; // close bit field

    *_pg_bytecount = _pg_byteindex;

}// encodeESC_TelemetryPackets_t

/*!
 * \brief Decode a ESC_TelemetryPackets_t structure from a byte array
 *
 * These bits are used to determine which packets are automatically transmitted
 * as telemetry data by the ESC. Only the packets described here can be
 * configured as telemetry packets
 * \param _pg_data points to the byte array to decoded data from
 * \param _pg_bytecount points to the starting location in the byte array, and will be incremented by the number of bytes decoded
 * \param _pg_user is the data to decode from the byte array
 * \return 1 if the data are decoded, else 0. If 0 is returned _pg_bytecount will not be updated.
 */
int decodeESC_TelemetryPackets_t(const uint8_t* _pg_data, int* _pg_bytecount, ESC_TelemetryPackets_t* _pg_user)
{
    int _pg_byteindex = *_pg_bytecount;

    // If this bit is set, the STATUS_A packet will be transmitted at the configured rate
    _pg_user->statusA = (_pg_data[_pg_byteindex] >> 7);

    // If this bit is set, the STATUS_B packet will be transmitted at the configured rate
    _pg_user->statusB = ((_pg_data[_pg_byteindex] >> 6) & 0x1);

    // If this bit is set, the STATUS_C packet will be transmitted at the configured rate
    _pg_user->statusC = ((_pg_data[_pg_byteindex] >> 5) & 0x1);

    // If this bit is set, the ACCELEROMETER packet will be transmitted at the configured rate
    _pg_user->accelerometer = ((_pg_data[_pg_byteindex] >> 4) & 0x1);

    // If this bit is set, the STATUS_D packet will be transmitted at the configured rate
    _pg_user->statusD = ((_pg_data[_pg_byteindex] >> 3) & 0x1);

    // Reserved for future use
    _pg_user->reservedB = ((_pg_data[_pg_byteindex] >> 2) & 0x1);

    // If this bit is set, any STATUS_x packets selected for telemetry will be mirrored on the Piccolo Downlink packet group (0x14)
    _pg_user->piccoloDownlink = ((_pg_data[_pg_byteindex] >> 1) & 0x1);

    // Reserved for future use
    _pg_user->reservedD = ((_pg_data[_pg_byteindex]) & 0x1);
    _pg_byteindex += 1; // close bit field

    *_pg_bytecount = _pg_byteindex;

    return 1;

}// decodeESC_TelemetryPackets_t

/*!
 * \brief Set a ESC_TelemetryPackets_t structure to initial values.
 *
 * Set a ESC_TelemetryPackets_t structure to initial values. Not all fields are set,
 * only those which the protocol specifies.
 * \param _pg_user is the structure whose data are set to initial values
 */
void initESC_TelemetryPackets_t(ESC_TelemetryPackets_t* _pg_user)
{

    // If this bit is set, the STATUS_A packet will be transmitted at the configured rate
    _pg_user->statusA = 1;

    // If this bit is set, the STATUS_B packet will be transmitted at the configured rate
    _pg_user->statusB = 1;

    // If this bit is set, the STATUS_C packet will be transmitted at the configured rate
    _pg_user->statusC = 1;

    // If this bit is set, the ACCELEROMETER packet will be transmitted at the configured rate
    _pg_user->accelerometer = 0;

}// initESC_TelemetryPackets_t

// end of ESCDefines.c
