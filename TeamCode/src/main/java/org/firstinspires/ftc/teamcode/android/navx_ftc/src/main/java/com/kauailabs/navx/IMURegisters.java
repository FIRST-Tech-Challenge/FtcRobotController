/* ============================================
 NavX-MXP source code is placed under the MIT license
 Copyright (c) 2015 Kauai Labs

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */
package org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx;

public class IMURegisters {

	/**********************************************/
	/* Device Identification Registers            */
	/**********************************************/

	public static final byte NAVX_REG_WHOAMI 			= 0x00; /* IMU_MODEL_XXX */
	public static final byte NAVX_REG_HW_REV			= 0x01;
	public static final byte NAVX_REG_FW_VER_MAJOR		= 0x02;
	public static final byte NAVX_REG_FW_VER_MINOR 		= 0x03;

	/**********************************************/
	/* Status and Control Registers               */
	/**********************************************/

	/* Read-write */
	public static final byte NAVX_REG_UPDATE_RATE_HZ	= 0x04; /* Range:  4 - 50 [unsigned byte] */
	/* Read-only */
	/* Accelerometer Full-Scale Range:  in units of G [unsigned byte] */
	public static final byte NAVX_REG_ACCEL_FSR_G		= 0x05;
	/* Gyro Full-Scale Range (Degrees/Sec):  Range:  250, 500, 1000 or 2000 [unsigned short] */
	public static final byte NAVX_REG_GYRO_FSR_DPS_L	= 0x06; /* Lower 8-bits of Gyro Full-Scale Range */
	public static final byte NAVX_REG_GYRO_FSR_DPS_H	= 0x07; /* Upper 8-bits of Gyro Full-Scale Range */
	public static final byte NAVX_REG_OP_STATUS       	= 0x08; /* NAVX_OP_STATUS_XXX */
	public static final byte NAVX_REG_CAL_STATUS       	= 0x09; /* NAVX_CAL_STATUS_XXX */
	public static final byte NAVX_REG_SELFTEST_STATUS 	= 0x0A; /* NAVX_SELFTEST_STATUS_XXX */
	public static final byte NAVX_REG_CAPABILITY_FLAGS_L = 0x0B;
	public static final byte NAVX_REG_CAPABILITY_FLAGS_H = 0x0C;

	/**********************************************/
	/* Processed Data Registers                   */
	/**********************************************/

	public static final byte NAVX_REG_SENSOR_STATUS_L	= 0x10; /* NAVX_SENSOR_STATUS_XXX */
	public static final byte NAVX_REG_SENSOR_STATUS_H	= 0x11;
	/* Timestamp:  [unsigned long] */
	public static final byte NAVX_REG_TIMESTAMP_L_L     = 0x12;
	public static final byte NAVX_REG_TIMESTAMP_L_H     = 0x13;
	public static final byte NAVX_REG_TIMESTAMP_H_L		= 0x14;
	public static final byte NAVX_REG_TIMESTAMP_H_H		= 0x15;

	/* Yaw, Pitch, Roll:  Range: -180.00 to 180.00 [signed hundredths] */
	/* Compass Heading:   Range: 0.00 to 360.00 [unsigned hundredths] */
	/* Altitude in Meters:  In units of meters [16:16] */

	public static final byte NAVX_REG_YAW_L 			= 0x16; /* Lower 8 bits of Yaw     */
	public static final byte NAVX_REG_YAW_H 			= 0x17; /* Upper 8 bits of Yaw     */
	public static final byte NAVX_REG_ROLL_L 			= 0x18; /* Lower 8 bits of Roll    */
	public static final byte NAVX_REG_ROLL_H 			= 0x19; /* Upper 8 bits of Roll    */
	public static final byte NAVX_REG_PITCH_L 			= 0x1A; /* Lower 8 bits of Pitch   */
	public static final byte NAVX_REG_PITCH_H 			= 0x1B; /* Upper 8 bits of Pitch   */
	public static final byte NAVX_REG_HEADING_L 		= 0x1C; /* Lower 8 bits of Heading */
	public static final byte NAVX_REG_HEADING_H 		= 0x1D; /* Upper 8 bits of Heading */
	public static final byte NAVX_REG_FUSED_HEADING_L	= 0x1E; /* Upper 8 bits of Fused Heading */
	public static final byte NAVX_REG_FUSED_HEADING_H	= 0x1F; /* Upper 8 bits of Fused Heading */
	public static final byte NAVX_REG_ALTITUDE_I_L		= 0x20;
	public static final byte NAVX_REG_ALTITUDE_I_H		= 0x21;
	public static final byte NAVX_REG_ALTITUDE_D_L		= 0x22;
	public static final byte NAVX_REG_ALTITUDE_D_H		= 0x23;

	/* World-frame Linear Acceleration: In units of +/- G * 1000 [signed thousandths] */

	public static final byte NAVX_REG_LINEAR_ACC_X_L	= 0x24; /* Lower 8 bits of Linear Acceleration X */
	public static final byte NAVX_REG_LINEAR_ACC_X_H	= 0x25; /* Upper 8 bits of Linear Acceleration X */
	public static final byte NAVX_REG_LINEAR_ACC_Y_L	= 0x26; /* Lower 8 bits of Linear Acceleration Y */
	public static final byte NAVX_REG_LINEAR_ACC_Y_H	= 0x27; /* Upper 8 bits of Linear Acceleration Y */
	public static final byte NAVX_REG_LINEAR_ACC_Z_L	= 0x28; /* Lower 8 bits of Linear Acceleration Z */
	public static final byte NAVX_REG_LINEAR_ACC_Z_H	= 0x29; /* Upper 8 bits of Linear Acceleration Z */

	/* Quaternion:  Range -1 to 1 [signed short ratio] */

	public static final byte NAVX_REG_QUAT_W_L 			= 0x2A; /* Lower 8 bits of Quaternion W */
	public static final byte NAVX_REG_QUAT_W_H 			= 0x2B; /* Upper 8 bits of Quaternion W */
	public static final byte NAVX_REG_QUAT_X_L 			= 0x2C; /* Lower 8 bits of Quaternion X */
	public static final byte NAVX_REG_QUAT_X_H 			= 0x2D; /* Upper 8 bits of Quaternion X */
	public static final byte NAVX_REG_QUAT_Y_L 			= 0x2E; /* Lower 8 bits of Quaternion Y */
	public static final byte NAVX_REG_QUAT_Y_H 			= 0x2F; /* Upper 8 bits of Quaternion Y */
	public static final byte NAVX_REG_QUAT_Z_L 			= 0x30; /* Lower 8 bits of Quaternion Z */
	public static final byte NAVX_REG_QUAT_Z_H 			= 0x31; /* Upper 8 bits of Quaternion Z */

	/**********************************************/
	/* Raw Data Registers                         */
	/**********************************************/

	/* Sensor Die Temperature:  Range +/- 150, In units of Centigrade * 100 [signed hundredths float */

	public static final byte NAVX_REG_MPU_TEMP_C_L		= 0x32; /* Lower 8 bits of Temperature */
	public static final byte NAVX_REG_MPU_TEMP_C_H		= 0x33; /* Upper 8 bits of Temperature */

	/* Raw, Calibrated Angular Rotation, in device units.  Value in DPS = units / GYRO_FSR_DPS [signed short] */

	public static final byte NAVX_REG_GYRO_X_L			= 0x34;
	public static final byte NAVX_REG_GYRO_X_H			= 0x35;
	public static final byte NAVX_REG_GYRO_Y_L			= 0x36;
	public static final byte NAVX_REG_GYRO_Y_H			= 0x37;
	public static final byte NAVX_REG_GYRO_Z_L			= 0x38;
	public static final byte NAVX_REG_GYRO_Z_H			= 0x39;

	/* Raw, Calibrated, Acceleration Data, in device units.  Value in G = units / ACCEL_FSR_G [signed short] */

	public static final byte NAVX_REG_ACC_X_L			= 0x3A;
	public static final byte NAVX_REG_ACC_X_H			= 0x3B;
	public static final byte NAVX_REG_ACC_Y_L			= 0x3C;
	public static final byte NAVX_REG_ACC_Y_H			= 0x3D;
	public static final byte NAVX_REG_ACC_Z_L			= 0x3E;
	public static final byte NAVX_REG_ACC_Z_H			= 0x3F;

	/* Raw, Calibrated, Un-tilt corrected Magnetometer Data, in device units.  1 unit = 0.15 uTesla [signed short] */

	public static final byte NAVX_REG_MAG_X_L			= 0x40;
	public static final byte NAVX_REG_MAG_X_H			= 0x41;
	public static final byte NAVX_REG_MAG_Y_L			= 0x42;
	public static final byte NAVX_REG_MAG_Y_H			= 0x43;
	public static final byte NAVX_REG_MAG_Z_L			= 0x44;
	public static final byte NAVX_REG_MAG_Z_H			= 0x45;

	/* Calibrated Pressure in millibars Valid Range:  10.00 Max:  1200.00 [16:16 float]  */

	public static final byte NAVX_REG_PRESSURE_IL       = 0x46;
	public static final byte NAVX_REG_PRESSURE_IH       = 0x47;
	public static final byte NAVX_REG_PRESSURE_DL       = 0x48;
	public static final byte NAVX_REG_PRESSURE_DH		= 0x49;

	/* Pressure Sensor Die Temperature:  Range +/- 150.00C [signed hundredths] */

	public static final byte NAVX_REG_PRESSURE_TEMP_L	= 0x4A;
	public static final byte NAVX_REG_PRESSURE_TEMP_H	= 0x4B;

	/**********************************************/
	/* Calibration Registers                      */
	/**********************************************/

	/* Yaw Offset: Range -180.00 to 180.00 [signed hundredths] */

	public static final byte NAVX_REG_YAW_OFFSET_L		= 0x4C; /* Lower 8 bits of Yaw Offset */
	public static final byte NAVX_REG_YAW_OFFSET_H		= 0x4D; /* Upper 8 bits of Yaw Offset */

	/* Quaternion Offset:  Range: -1 to 1 [signed short ratio]  */

	public static final byte NAVX_REG_QUAT_OFFSET_W_L 	= 0x4E; /* Lower 8 bits of Quaternion W */
	public static final byte NAVX_REG_QUAT_OFFSET_W_H 	= 0x4F; /* Upper 8 bits of Quaternion W */
	public static final byte NAVX_REG_QUAT_OFFSET_X_L 	= 0x50; /* Lower 8 bits of Quaternion X */
	public static final byte NAVX_REG_QUAT_OFFSET_X_H 	= 0x51; /* Upper 8 bits of Quaternion X */
	public static final byte NAVX_REG_QUAT_OFFSET_Y_L 	= 0x52; /* Lower 8 bits of Quaternion Y */
	public static final byte NAVX_REG_QUAT_OFFSET_Y_H 	= 0x53; /* Upper 8 bits of Quaternion Y */
	public static final byte NAVX_REG_QUAT_OFFSET_Z_L 	= 0x54; /* Lower 8 bits of Quaternion Z */
	public static final byte NAVX_REG_QUAT_OFFSET_Z_H 	= 0x55; /* Upper 8 bits of Quaternion Z */

	/**********************************************/
	/* Integrated Data Registers                  */
	/**********************************************/

	/* Integration Control (Write-Only)           */
	public static final byte NAVX_REG_INTEGRATION_CTL	= 0x56;
	public static final byte NAVX_REG_PAD_UNUSED        = 0x57;

	/* Velocity:  Range -32768.9999 - 32767.9999 in units of Meters/Sec      */

	public static final byte NAVX_REG_VEL_X_I_L         = 0x58;
	public static final byte NAVX_REG_VEL_X_I_H         = 0x59;
	public static final byte NAVX_REG_VEL_X_D_L         = 0x5A;
	public static final byte NAVX_REG_VEL_X_D_H         = 0x5B;
	public static final byte NAVX_REG_VEL_Y_I_L         = 0x5C;
	public static final byte NAVX_REG_VEL_Y_I_H         = 0x5D;
	public static final byte NAVX_REG_VEL_Y_D_L         = 0x5E;
	public static final byte NAVX_REG_VEL_Y_D_H         = 0x5F;
	public static final byte NAVX_REG_VEL_Z_I_L         = 0x60;
	public static final byte NAVX_REG_VEL_Z_I_H         = 0x61;
	public static final byte NAVX_REG_VEL_Z_D_L         = 0x62;
	public static final byte NAVX_REG_VEL_Z_D_H         = 0x63;

	/* Displacement:  Range -32768.9999 - 32767.9999 in units of Meters      */

	public static final byte NAVX_REG_DISP_X_I_L        = 0x64;
	public static final byte NAVX_REG_DISP_X_I_H        = 0x65;
	public static final byte NAVX_REG_DISP_X_D_L        = 0x66;
	public static final byte NAVX_REG_DISP_X_D_H        = 0x67;
	public static final byte NAVX_REG_DISP_Y_I_L        = 0x68;
	public static final byte NAVX_REG_DISP_Y_I_H        = 0x69;
	public static final byte NAVX_REG_DISP_Y_D_L        = 0x6A;
	public static final byte NAVX_REG_DISP_Y_D_H        = 0x6B;
	public static final byte NAVX_REG_DISP_Z_I_L        = 0x6C;
	public static final byte NAVX_REG_DISP_Z_I_H        = 0x6D;
	public static final byte NAVX_REG_DISP_Z_D_L        = 0x6E;
	public static final byte NAVX_REG_DISP_Z_D_H        = 0x6F;
	
	public static final byte NAVX_REG_LAST = NAVX_REG_DISP_Z_D_H;
}
