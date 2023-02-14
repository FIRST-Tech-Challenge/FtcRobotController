/* ============================================
 NavX-MXP and NavX-Micro source code is placed under the MIT license
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
package org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc;

import java.util.Arrays;



import android.os.Process;
import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.TimestampedData;

import org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.AHRSProtocol;
import org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.IMUProtocol;
import org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.IMURegisters;

/**
 * The AHRS class provides an interface to AHRS capabilities
 * of the KauaiLabs navX2 and navX Robotics Navigation Sensor via I2Cs on the Android-
 * based FTC robotics control system, where communications occur via an I2C
 * port on the REV Expansion Hub or REV Control Hub.
 *
 * The AHRS class enables access to basic connectivity and state information,
 * as well as key 6-axis and 9-axis orientation information (yaw, pitch, roll,
 * compass heading, fused (9-axis) heading and magnetic disturbance detection.
 *
 * Additionally, the AHRS class also provides access to extended information
 * including linear acceleration, motion detection, rotation detection and sensor
 * temperature.
 *
 * If used with navX-Aero-enabled devices, the AHRS class also provides access to
 * altitude, barometric pressure and pressure sensor temperature data
 * @author Scott
 */
public class AHRS {

    /**
     * Identifies one of the three sensing axes on the navX sensor board.  Note that these axes are
     * board-relative ("Board Frame"), and are not necessarily the same as the logical axes of the
     * chassis on which the sensor is mounted.
     *
     * For more information on sensor orientation, please see the navX sensor
     * <a href=http://navx-micro.kauailabs.com//installation/orientation/>Orientation</a> page.
     */
    public enum BoardAxis {
        kBoardAxisX(0),
        kBoardAxisY(1),
        kBoardAxisZ(2);

        private int value;

        private BoardAxis(int value) {
            this.value = value;
        }
        public int getValue() {
            return this.value;
        }
    };

    /**
     * Indicates which sensor board axis is used as the "yaw" (gravity) axis.
     *
     * This selection may be modified via the <a href=http://navx-micro.kauailabs.com/installation/omnimount/>Omnimount</a> feature.
     *
     */
    static public class BoardYawAxis
    {
        public BoardAxis board_axis;
        public boolean up;
    };

    /**
     * The DeviceDataType specifies the
     * type of data to be retrieved from the sensor.  Due to limitations in the
     * communication bandwidth, only a subset of all available data can be streamed
     * and still maintain a 50Hz update rate via the Core Device Interface Module,
     * since it is limited to a maximum of one 26-byte transfer every 10ms.
     * Note that if all data types are required,
     */
    public enum DeviceDataType {
        /**
         * (default):  All 6 and 9-axis processed data, sensor status and timestamp
         */
        kProcessedData(0),
        /**
         * Unit Quaternions and unprocessed data from each individual sensor.  Note that
         * this data does not include a timestamp.  If a timestamp is needed, use
         * the kAll flag instead.
         */
        kQuatAndRawData(1),
        /**
         * All processed and raw data, sensor status and timestamps.  Note that on a
         * Android-based FTC robot using the "Core Device Interface Module", acquiring
         * all data requires to I2C transactions.
         */
        kAll(3);

        private int value;

        private DeviceDataType(int value){
            this.value = value;
        }

        public int getValue(){
            return this.value;
        }
    };

    private class BoardState {
        public short capability_flags;
        public byte  update_rate_hz;
        public short accel_fsr_g;
        public short gyro_fsr_dps;
        public byte  op_status;
        public byte  cal_status;
        public byte  selftest_status;
    }

    private static AHRS instance = null;
    private static boolean enable_logging = false;
    private static final int NAVX_DEFAULT_UPDATE_RATE_HZ = 50;

    private NavxMicroNavigationSensor sensor = null;
    private navXIOThread io_thread_obj;
    private Thread io_thread;
    private int update_rate_hz = NAVX_DEFAULT_UPDATE_RATE_HZ;
    private IDataArrivalSubscriber callbacks[];
    private final int MAX_NUM_CALLBACKS = 3;

    AHRSProtocol.AHRSPosUpdate curr_data;
    BoardState board_state;
    AHRSProtocol.BoardID board_id;
    IMUProtocol.GyroUpdate raw_data_update;

    protected AHRS(NavxMicroNavigationSensor sensor,
                   DeviceDataType data_type, int update_rate_hz) {
        this.callbacks = new IDataArrivalSubscriber[MAX_NUM_CALLBACKS];
        this.sensor = sensor;
        this.update_rate_hz = update_rate_hz;
        this.curr_data = new AHRSProtocol.AHRSPosUpdate();
        this.board_state = new BoardState();
        this.board_id = new AHRSProtocol.BoardID();
        this.raw_data_update = new IMUProtocol.GyroUpdate();

        io_thread_obj   = new navXIOThread(update_rate_hz, data_type, curr_data);
        io_thread_obj.start();
        io_thread       = new Thread(io_thread_obj);

        io_thread.start();
    }

    /**
     * Registers a callback interface.  This interface
     * will be called back when new data is available,
     * based upon a change in the sensor timestamp.
     *<p>
     * Note that this callback will occur within the context of the
     * device IO thread, which is not the same thread context the
     * caller typically executes in.
     */
    public boolean registerCallback( IDataArrivalSubscriber callback ) {
        boolean registered = false;
        for ( int i = 0; i < this.callbacks.length; i++ ) {
            if (this.callbacks[i] == null) {
                this.callbacks[i] = callback;
                registered = true;
                break;
            }
        }
        return registered;
    }

    /**
     * Deregisters a previously registered callback interface.
     *
     * Be sure to deregister any callback which have been
     * previously registered, to ensure that the object
     * implementing the callback interface does not continue
     * to be accessed when no longer necessary.
     */
    public boolean deregisterCallback( IDataArrivalSubscriber callback ) {
        boolean deregistered = false;
        for ( int i = 0; i < this.callbacks.length; i++ ) {
            if (this.callbacks[i] == callback) {
                this.callbacks[i] = null;
                deregistered = true;
                break;
            }
        }
        return deregistered;
    }

    public void close() {
        io_thread_obj.stop();
        for ( int i = 0; i < callbacks.length; i++ ) {
            callbacks[i] = null;
        }
        try {
            io_thread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        instance = null;
    }

    /**
     * Returns the single instance (singleton) of the AHRS class.  If the singleton
     * does not alrady exist, it will be created using the parameters passed in.
     * The default update rate will be used.
     *
     * If the singleton already exists, the parameters passed in will be ignored.
     * @return The singleton AHRS class instance.
     */
    public static AHRS getInstance(NavxMicroNavigationSensor sensor,
                                   DeviceDataType data_type) {
        if (instance == null) {
            instance = new AHRS(sensor, data_type, NAVX_DEFAULT_UPDATE_RATE_HZ);
        }
        return instance;
    }

    /**
     * Returns the single instance (singleton) of the AHRS class.  If the singleton
     * does not alrady exist, it will be created using the parameters passed in,
     * including a custom update rate.  Use this function if an update rate other than
     * the default is desired.
     *
     * NOTE:  The range of valid requested update rates is from 4 to 66.  However, the
     * actual update does not always match the requested update rate.  The following table
     * summarizes the requested to actual update rate lookup table:
     *
     *   Actual       Requested
     *   66.6         58-66
     *   50           45-57
     *   40           37-44
     *   33.3         31-36
     *   28.57        27-30
     *   25           25-26
     *   22.22        22-23
     *   20           20-21
     *   18.18        18-19
     *   16.67        17
     *   15.38        15-16
     *   14.28        14
     *
     * Requested values below 14Hz result in an actual rate which is accurate to within 1Hz.
     *
     * The reason for this difference is that the update rate must be an even multiple of
     * a 200Hz clock (5ms).  So an actual of 66.6 (15ms/sample) can be calculated as follows:
     *      actual_rate = 200 / (200 / requested_rate)
     *
     * The getActualUpdateRate() can be used to calculate this value.
     *
     * @return The singleton AHRS class instance.
     */
    public static AHRS getInstance(NavxMicroNavigationSensor sensor,
                                   DeviceDataType data_type, byte update_rate_hz) {
        if (instance == null) {
            instance = new AHRS(sensor, data_type, update_rate_hz);
        }
        return instance;
    }

    /* Configures the AHRS class logging.  To enable logging,
     * the input parameter should be set to 'true'.
     */
    public static void setLogging( boolean enabled ) {
        enable_logging = enabled;
    }

    /* Returns 'true' if AHRS class logging is enabled, otherwise
     * returns 'false'.
     */
    public static boolean getLogging() {
        return enable_logging;
    }

    /* Returns the currently configured DeviceDataType.
     */
    public DeviceDataType getDeviceDataType() {
        return this.io_thread_obj.data_type;
    }

    /**
     * Returns the current pitch value (in degrees, from -180 to 180)
     * reported by the sensor.  Pitch is a measure of rotation around
     * the X Axis.
     * @return The current pitch value in degrees (-180 to 180).
     */
    public float getPitch() {
        return curr_data.pitch;
    }

    /**
     * Returns the current roll value (in degrees, from -180 to 180)
     * reported by the sensor.  Roll is a measure of rotation around
     * the X Axis.
     * @return The current roll value in degrees (-180 to 180).
     */
    public float getRoll() {
        return curr_data.roll;
    }

    /**
     * Returns the current yaw value (in degrees, from -180 to 180)
     * reported by the sensor.  Yaw is a measure of rotation around
     * the Z Axis (which is perpendicular to the earth).
     *<p>
     * Note that the returned yaw value will be offset by a user-specified
     * offset value; this user-specified offset value is set by
     * invoking the zeroYaw() method.
     * @return The current yaw value in degrees (-180 to 180).
     */
    public float getYaw() {
        return curr_data.yaw;
    }

    /**
     * Returns the current tilt-compensated compass heading
     * value (in degrees, from 0 to 360) reported by the sensor.
     *<p>
     * Note that this value is sensed by a magnetometer,
     * which can be affected by nearby magnetic fields (e.g., the
     * magnetic fields generated by nearby motors).
     *<p>
     * Before using this value, ensure that (a) the magnetometer
     * has been calibrated and (b) that a magnetic disturbance is
     * not taking place at the instant when the compass heading
     * was generated.
     * @return The current tilt-compensated compass heading, in degrees (0-360).
     */
    public float getCompassHeading() {
        return curr_data.compass_heading;
    }

    /**
     * Sets the user-specified yaw offset to the current
     * yaw value reported by the sensor.
     *<p>
     * This user-specified yaw offset is automatically
     * subtracted from subsequent yaw values reported by
     * the getYaw() method.
     */
    public void zeroYaw() {
        io_thread_obj.zeroYaw();
    }

    /**
     * Returns true if the sensor is currently performing automatic
     * gyro/accelerometer calibration.  Automatic calibration occurs
     * when the sensor is initially powered on, during which time the
     * sensor should be held still, with the Z-axis pointing up
     * (perpendicular to the earth).
     *<p>
     * NOTE:  During this automatic calibration, the yaw, pitch and roll
     * values returned may not be accurate.
     *<p>
     * Once calibration is complete, the sensor will automatically remove
     * an internal yaw offset value from all reported values.
     *<p>
     * @return Returns true if the sensor is currently automatically
     * calibrating the gyro and accelerometer sensors.
     */

    public boolean isCalibrating() {
        return !((curr_data.cal_status &
                AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_STATE_MASK) ==
                AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_COMPLETE);
    }

    /**
     * Indicates whether the sensor is currently connected
     * to the host computer.  A connection is considered established
     * whenever communication with the sensor has occurred recently.
     *<p>
     * @return Returns true if a valid update has been recently received
     * from the sensor.
     */

    public boolean isConnected() {
        return io_thread_obj.isConnected();
    }

    /**
     * Returns the navX-Model device's currently configured update
     * rate.  Note that the update rate that can actually be realized
     * is a value evenly divisible by the navX-Model device's internal
     * motion processor sample clock (200Hz).  Therefore, the rate that
     * is returned may be lower than the requested sample rate.
     *
     * The actual sample rate is rounded down to the nearest integer
     * that is divisible by the number of Digital Motion Processor clock
     * ticks.  For instance, a request for 58 Hertz will result in
     * an actual rate of 66Hz (200 / (200 / 58), using integer
     * math.
     *
     * @return Returns the current actual update rate in Hz
     * (cycles per second).
     */

    public byte getActualUpdateRate() {
        final int NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ = 200;
        int integer_update_rate =  board_state.update_rate_hz;
        int realized_update_rate = NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ /
                (NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ / integer_update_rate);
        return (byte)realized_update_rate;
    }

    /**
     * Returns the current number of data samples being transferred
     * from the navX-Model device in the last second.  Note that this
     * number may be greater than the sensors update rate.
     *
     * @return Returns the count of data samples received in the
     * last second.
     */

    public int getCurrentTransferRate() {
        return this.io_thread_obj.last_second_hertz;
    }

    /* Returns the number of navX-Model processed data samples
     * that were retrieved from the sensor that had a sensor
     * timestamp value equal to the timestamp of the last
     * sensor data sample.
     *
     * This information can be used to match the navX-Model
     * sensor update rate w/the effective update rate which
     * can be achieved in the Robot Controller, taking into
     * account the communication over the network with the
     * Core Device Interface Module.
     */

    public int getDuplicateDataCount() {
        return this.io_thread_obj.getDuplicateDataCount();
    }

    /**
     * Returns the count in bytes of data received from the
     * sensor.  This could can be useful for diagnosing
     * connectivity issues.
     *<p>
     * If the byte count is increasing, but the update count
     * (see getUpdateCount()) is not, this indicates a software
     * misconfiguration.
     * @return The number of bytes received from the sensor.
     */
    public double getByteCount() {
        return io_thread_obj.getByteCount();
    }

    /**
     * Returns the count of valid updates which have
     * been received from the sensor.  This count should increase
     * at the same rate indicated by the configured update rate.
     * @return The number of valid updates received from the sensor.
     */
    public double getUpdateCount() {
        return io_thread_obj.getUpdateCount();
    }

    /**
     * Returns the current linear acceleration in the X-axis (in G).
     *<p>
     * World linear acceleration refers to raw acceleration data, which
     * has had the gravity component removed, and which has been rotated to
     * the same reference frame as the current yaw value.  The resulting
     * value represents the current acceleration in the x-axis of the
     * body (e.g., the robot) on which the sensor is mounted.
     *<p>
     * @return Current world linear acceleration in the X-axis (in G).
     */
    public float getWorldLinearAccelX()
    {
        return curr_data.linear_accel_x;
    }

    /**
     * Returns the current linear acceleration in the Y-axis (in G).
     *<p>
     * World linear acceleration refers to raw acceleration data, which
     * has had the gravity component removed, and which has been rotated to
     * the same reference frame as the current yaw value.  The resulting
     * value represents the current acceleration in the Y-axis of the
     * body (e.g., the robot) on which the sensor is mounted.
     *<p>
     * @return Current world linear acceleration in the Y-axis (in G).
     */
    public float getWorldLinearAccelY()
    {
        return curr_data.linear_accel_y;
    }

    /**
     * Returns the current linear acceleration in the Z-axis (in G).
     *<p>
     * World linear acceleration refers to raw acceleration data, which
     * has had the gravity component removed, and which has been rotated to
     * the same reference frame as the current yaw value.  The resulting
     * value represents the current acceleration in the Z-axis of the
     * body (e.g., the robot) on which the sensor is mounted.
     *<p>
     * @return Current world linear acceleration in the Z-axis (in G).
     */
    public float getWorldLinearAccelZ()
    {
        return curr_data.linear_accel_z;
    }

    /**
     * Indicates if the sensor is currently detecting motion,
     * based upon the X and Y-axis world linear acceleration values.
     * If the sum of the absolute values of the X and Y axis exceed
     * a "motion threshold", the motion state is indicated.
     *<p>
     * @return Returns true if the sensor is currently detecting motion.
     */
    public boolean isMoving()
    {
        return (curr_data.sensor_status &
                AHRSProtocol.NAVX_SENSOR_STATUS_MOVING) != 0;
    }

    /**
     * Indicates if the sensor is currently detecting yaw rotation,
     * based upon whether the change in yaw over the last second
     * exceeds the "Rotation Threshold."
     *<p>
     * Yaw Rotation can occur either when the sensor is rotating, or
     * when the sensor is not rotating AND the current gyro calibration
     * is insufficiently calibrated to yield the standard yaw drift rate.
     *<p>
     * @return Returns true if the sensor is currently detecting motion.
     */
    public boolean isRotating()
    {
        return !((curr_data.sensor_status &
                AHRSProtocol.NAVX_SENSOR_STATUS_YAW_STABLE) != 0);
    }

    /**
     * Returns the current altitude, based upon calibrated readings
     * from a barometric pressure sensor, and the currently-configured
     * sea-level barometric pressure [navX Aero only].  This value is in units of meters.
     *<p>
     * NOTE:  This value is only valid sensors including a pressure
     * sensor.  To determine whether this value is valid, see
     * isAltitudeValid().
     *<p>
     * @return Returns current altitude in meters (as long as the sensor includes
     * an installed on-board pressure sensor).
     */
    public float getAltitude()
    {
        return curr_data.altitude;
    }

    /**
     * Indicates whether the current altitude (and barometric pressure) data is
     * valid. This value will only be true for a sensor with an onboard
     * pressure sensor installed.
     *<p>
     * If this value is false for a board with an installed pressure sensor,
     * this indicates a malfunction of the onboard pressure sensor.
     *<p>
     * @return Returns true if a working pressure sensor is installed.
     */
    public boolean isAltitudeValid()
    {
        return (curr_data.sensor_status &
                AHRSProtocol.NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0;
    }

    /**
     * Returns the "fused" (9-axis) heading.
     *<p>
     * The 9-axis heading is the fusion of the yaw angle, the tilt-corrected
     * compass heading, and magnetic disturbance detection.  Note that the
     * magnetometer calibration procedure is required in order to
     * achieve valid 9-axis headings.
     *<p>
     * The 9-axis Heading represents the sensor's best estimate of current heading,
     * based upon the last known valid Compass Angle, and updated by the change in the
     * Yaw Angle since the last known valid Compass Angle.  The last known valid Compass
     * Angle is updated whenever a Calibrated Compass Angle is read and the sensor
     * has recently rotated less than the Compass Noise Bandwidth (~2 degrees).
     * @return Fused Heading in Degrees (range 0-360)
     */
    public float getFusedHeading()
    {
        return curr_data.fused_heading;
    }

    /**
     * Indicates whether the current magnetic field strength diverges from the
     * calibrated value for the earth's magnetic field by more than the currently-
     * configured Magnetic Disturbance Ratio.
     *<p>
     * This function will always return false if the sensor's magnetometer has
     * not yet been calibrated; see isMagnetometerCalibrated().
     * @return true if a magnetic disturbance is detected (or the magnetometer is uncalibrated).
     */
    public boolean isMagneticDisturbance()
    {
        return (curr_data.sensor_status &
                AHRSProtocol.NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0;
    }

    /**
     * Indicates whether the magnetometer has been calibrated.
     *<p>
     * Magnetometer Calibration must be performed by the user.
     *<p>
     * Note that if this function does indicate the magnetometer is calibrated,
     * this does not necessarily mean that the calibration quality is sufficient
     * to yield valid compass headings.
     *<p>
     * @return Returns true if magnetometer calibration has been performed.
     */
    public boolean isMagnetometerCalibrated()
    {
        return (curr_data.cal_status &
                AHRSProtocol.NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0;
    }

    /* Unit Quaternions */

    /**
     * Returns the imaginary portion (W) of the Orientation Quaternion which
     * fully describes the current sensor orientation with respect to the
     * reference angle defined as the angle at which the yaw was last "zeroed".
     *<p>
     * Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2
     * to 2.  This total range (4) can be associated with a unit circle, since
     * each circle is comprised of 4 PI Radians.
     * <p>
     * For more information on Quaternions and their use, please see this <a href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>definition</a>.
     * @return Returns the imaginary portion (W) of the quaternion.
     */
    public float getQuaternionW() {
        return ((float)curr_data.quat_w / 16384.0f);
    }
    /**
     * Returns the real portion (X axis) of the Orientation Quaternion which
     * fully describes the current sensor orientation with respect to the
     * reference angle defined as the angle at which the yaw was last "zeroed".
     * <p>
     * Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2
     * to 2.  This total range (4) can be associated with a unit circle, since
     * each circle is comprised of 4 PI Radians.
     * <p>
     * For more information on Quaternions and their use, please see this <a href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>description</a>.
     * @return Returns the real portion (X) of the quaternion.
     */
    public float getQuaternionX() {
        return ((float)curr_data.quat_x / 16384.0f);
    }
    /**
     * Returns the real portion (X axis) of the Orientation Quaternion which
     * fully describes the current sensor orientation with respect to the
     * reference angle defined as the angle at which the yaw was last "zeroed".
     *
     * Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2
     * to 2.  This total range (4) can be associated with a unit circle, since
     * each circle is comprised of 4 PI Radians.
     *
     * For more information on Quaternions and their use, please see:
     *
     *   https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
     *
     * @return Returns the real portion (X) of the quaternion.
     */
    public float getQuaternionY() {
        return ((float)curr_data.quat_y / 16384.0f);
    }
    /**
     * Returns the real portion (X axis) of the Orientation Quaternion which
     * fully describes the current sensor orientation with respect to the
     * reference angle defined as the angle at which the yaw was last "zeroed".
     *
     * Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2
     * to 2.  This total range (4) can be associated with a unit circle, since
     * each circle is comprised of 4 PI Radians.
     *
     * For more information on Quaternions and their use, please see:
     *
     *   https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
     *
     * @return Returns the real portion (X) of the quaternion.
     */
    public float getQuaternionZ() {
        return ((float)curr_data.quat_z / 16384.0f);
    }

    /**
     * Returns the current temperature (in degrees centigrade) reported by
     * the sensor's gyro/accelerometer circuit.
     *<p>
     * This value may be useful in order to perform advanced temperature-
     * correction of raw gyroscope and accelerometer values.
     *<p>
     * @return The current temperature (in degrees centigrade).
     */
    public float getTempC()
    {
        return curr_data.mpu_temp;
    }

    /**
     * Returns information regarding which sensor board axis (X,Y or Z) and
     * direction (up/down) is currently configured to report Yaw (Z) angle
     * values.   NOTE:  If the board firmware supports Omnimount, the board yaw
     * axis/direction are configurable.
     *<p>
     * For more information on Omnimount, please see:
     *<p>
     * http://navx-mxp.kauailabs.com/navx-mxp/installation/omnimount/
     *<p>
     * @return The currently-configured board yaw axis/direction.
     */
    public BoardYawAxis getBoardYawAxis() {
        BoardYawAxis yaw_axis = new BoardYawAxis();
        short yaw_axis_info = (short)(board_state.capability_flags >> 3);
        yaw_axis_info &= 7;
        if ( yaw_axis_info == AHRSProtocol.OMNIMOUNT_DEFAULT) {
            yaw_axis.up = true;
            yaw_axis.board_axis = BoardAxis.kBoardAxisZ;
        } else {
            yaw_axis.up = (yaw_axis_info & 0x01) != 0;
            yaw_axis_info >>= 1;
            switch ( (byte)yaw_axis_info ) {
                case 0:
                    yaw_axis.board_axis = BoardAxis.kBoardAxisX;
                    break;
                case 1:
                    yaw_axis.board_axis = BoardAxis.kBoardAxisY;
                    break;
                case 2:
                default:
                    yaw_axis.board_axis = BoardAxis.kBoardAxisZ;
                    break;
            }
        }
        return yaw_axis;
    }

    /**
     * Returns the version number of the firmware currently executing
     * on the sensor.
     *<p>
     * To update the firmware to the latest version, please see:
     *<p>
     *   http://navx-mxp.kauailabs.com/navx-mxp/support/updating-firmware/
     *<p>
     * @return The firmware version in the format [MajorVersion].[MinorVersion]
     */
    public String getFirmwareVersion() {
        double version_number = (double)board_id.fw_ver_major;
        version_number += ((double)board_id.fw_ver_minor / 10);
        String fw_version = Double.toString(version_number);
        return fw_version;
    }

    private final float DEV_UNITS_MAX = 32768.0f;

    /**
     * Returns the current raw (unprocessed) X-axis gyro rotation rate (in degrees/sec).  NOTE:  this
     * value is un-processed, and should only be accessed by advanced users.
     * Typically, rotation about the X Axis is referred to as "Pitch".  Calibrated
     * and Integrated Pitch data is accessible via the {@link #getPitch()} method.
     *<p>
     * @return Returns the current rotation rate (in degrees/sec).
     */
    public float getRawGyroX() {
        return this.raw_data_update.gyro_x / (DEV_UNITS_MAX / (float)this.board_state.gyro_fsr_dps);
    }

    /**
     * Returns the current raw (unprocessed) Y-axis gyro rotation rate (in degrees/sec).  NOTE:  this
     * value is un-processed, and should only be accessed by advanced users.
     * Typically, rotation about the T Axis is referred to as "Roll".  Calibrated
     * and Integrated Pitch data is accessible via the {@link #getRoll()} method.
     *<p>
     * @return Returns the current rotation rate (in degrees/sec).
     */
    public float getRawGyroY() {
        return this.raw_data_update.gyro_y / (DEV_UNITS_MAX / (float)this.board_state.gyro_fsr_dps);
    }

    /**
     * Returns the current raw (unprocessed) Z-axis gyro rotation rate (in degrees/sec).  NOTE:  this
     * value is un-processed, and should only be accessed by advanced users.
     * Typically, rotation about the T Axis is referred to as "Yaw".  Calibrated
     * and Integrated Pitch data is accessible via the {@link #getYaw()} method.
     *<p>
     * @return Returns the current rotation rate (in degrees/sec).
     */
    public float getRawGyroZ() {
        return this.raw_data_update.gyro_z / (DEV_UNITS_MAX / (float)this.board_state.gyro_fsr_dps);
    }

    /**
     * Returns the current raw (unprocessed) X-axis acceleration rate (in G).  NOTE:  this
     * value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not had acceleration due to gravity removed from it, and has not been rotated to
     * the world reference frame.  Gravity-corrected, world reference frame-corrected
     * X axis acceleration data is accessible via the {@link #getWorldLinearAccelX()} method.
     *<p>
     * @return Returns the current acceleration rate (in G).
     */
    public float getRawAccelX() {
        return this.raw_data_update.accel_x / (DEV_UNITS_MAX / (float)this.board_state.accel_fsr_g);
    }

    /**
     * Returns the current raw (unprocessed) Y-axis acceleration rate (in G).  NOTE:  this
     * value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not had acceleration due to gravity removed from it, and has not been rotated to
     * the world reference frame.  Gravity-corrected, world reference frame-corrected
     * Y axis acceleration data is accessible via the {@link #getWorldLinearAccelY()} method.
     *<p>
     * @return Returns the current acceleration rate (in G).
     */
    public float getRawAccelY() {
        return this.raw_data_update.accel_y / (DEV_UNITS_MAX / (float)this.board_state.accel_fsr_g);
    }

    /**
     * Returns the current raw (unprocessed) Z-axis acceleration rate (in G).  NOTE:  this
     * value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not had acceleration due to gravity removed from it, and has not been rotated to
     * the world reference frame.  Gravity-corrected, world reference frame-corrected
     * Z axis acceleration data is accessible via the {@link #getWorldLinearAccelZ()} method.
     *<p>
     * @return Returns the current acceleration rate (in G).
     */
    public float getRawAccelZ() {
        return this.raw_data_update.accel_z / (DEV_UNITS_MAX / (float)this.board_state.accel_fsr_g);
    }

    private final float UTESLA_PER_DEV_UNIT = 0.15f;

    /**
     * Returns the current raw (unprocessed) X-axis magnetometer reading (in uTesla).  NOTE:
     * this value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not been tilt-corrected, and has not been combined with the other magnetometer axis
     * data to yield a compass heading.  Tilt-corrected compass heading data is accessible
     * via the {@link #getCompassHeading()} method.
     *<p>
     * @return Returns the mag field strength (in uTesla).
     */
    public float getRawMagX() {
        return this.raw_data_update.mag_x / UTESLA_PER_DEV_UNIT;
    }

    /**
     * Returns the current raw (unprocessed) Y-axis magnetometer reading (in uTesla).  NOTE:
     * this value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not been tilt-corrected, and has not been combined with the other magnetometer axis
     * data to yield a compass heading.  Tilt-corrected compass heading data is accessible
     * via the {@link #getCompassHeading()} method.
     *<p>
     * @return Returns the mag field strength (in uTesla).
     */
    public float getRawMagY() {
        return this.raw_data_update.mag_y / UTESLA_PER_DEV_UNIT;
    }

    /**
     * Returns the current raw (unprocessed) Z-axis magnetometer reading (in uTesla).  NOTE:
     * this value is unprocessed, and should only be accessed by advanced users.  This raw value
     * has not been tilt-corrected, and has not been combined with the other magnetometer axis
     * data to yield a compass heading.  Tilt-corrected compass heading data is accessible
     * via the {@link #getCompassHeading()} method.
     *<p>
     * @return Returns the mag field strength (in uTesla).
     */
    public float getRawMagZ() {
        return this.raw_data_update.mag_z / UTESLA_PER_DEV_UNIT;
    }

    /**
     * Returns the current barometric pressure (in millibar) [navX Aero only].
     *<p>
     *This value is valid only if a barometric pressure sensor is onboard.
     *
     * @return Returns the current barometric pressure (in millibar).
     */
    public float getPressure() {
        // TODO implement for navX-Aero.
        return 0;
    }

    class navXIOThread implements Runnable {

        int update_rate_hz;
        protected boolean keep_running;
        boolean request_zero_yaw;
        boolean is_connected;
        int byte_count;
        int update_count;
        DeviceDataType data_type;
        AHRSProtocol.AHRSPosUpdate ahrspos_update;
        long curr_sensor_timestamp;
        long last_valid_sensor_timestamp;
        int duplicate_sensor_data_count;
        int last_second_hertz;
        int hertz_counter;
        Object io_thread_event;
        Object reset_yaw_critical_section;

        final int NAVX_REGISTER_FIRST           = IMURegisters.NAVX_REG_WHOAMI;
        final int NAVX_REGISTER_PROC_FIRST      = IMURegisters.NAVX_REG_SENSOR_STATUS_L;
        final int NAVX_REGISTER_RAW_FIRST       = IMURegisters.NAVX_REG_QUAT_W_L;
        final int I2C_TIMEOUT_MS                = 500;

        public navXIOThread( int update_rate_hz, DeviceDataType data_type,
                             AHRSProtocol.AHRSPosUpdate ahrspos_update) {
            this.keep_running = false;
            this.update_rate_hz = update_rate_hz;
            this.request_zero_yaw = false;
            this.is_connected = false;
            this.byte_count = 0;
            this.update_count = 0;
            this.ahrspos_update = ahrspos_update;
            this.data_type = data_type;
            this.last_valid_sensor_timestamp = 0;
            this.duplicate_sensor_data_count = 0;
            this.last_second_hertz = 0;
            this.hertz_counter = 0;
            this.io_thread_event = new Object();
            this.reset_yaw_critical_section = new Object();

            Process.setThreadPriority(Process.THREAD_PRIORITY_MORE_FAVORABLE);
        }

        public void start() {
            keep_running = true;
        }
        public void stop() {
            keep_running = false;
            signalThread();
        }

        public void zeroYaw() {
            synchronized(reset_yaw_critical_section) {
                request_zero_yaw = true;
                /* Notify all data subscribers that the yaw
                   is about to be reset.  This informs clients (e.g., a PID Controller)
                   that a yaw discontinuity is occurring; they should receive this
                   notification before the next valid sample (which should reflect
                   the new zero value) is received/delivered.
                 */
                for ( int i = 0; i < callbacks.length; i++ ) {
                    IDataArrivalSubscriber callback = callbacks[i];
                    if (callback != null) {
                        callback.yawReset();
                    }
                }
            }
            signalThread();
        }

        public int getByteCount() {
            return byte_count;
        }

        public int getDuplicateDataCount() {
            return duplicate_sensor_data_count;
        }

        public void addToByteCount( int new_byte_count ) {
            byte_count += new_byte_count;
        }

        public int getUpdateCount() {
            return update_count;
        }

        public void incrementUpdateCount() {
            update_count++;
        }

        public boolean isConnected() {
            return is_connected;
        }

        public void setConnected( boolean new_connected ) {
            is_connected = new_connected;
            if ( !is_connected ) {
                signalThread();
            }
        }

        private void signalThread() {
            synchronized (io_thread_event) {
                io_thread_event.notify();
            }
        }

        @Override
        public void run() {

            final int DIM_MAX_I2C_READ_LEN          = 26;   // TODO:  IS this still valid?
            Boolean i2c_timeout                     = false;

            byte[] update_rate_command  = new byte[1];
            update_rate_command[0]      = (byte)update_rate_hz;
            byte[] zero_yaw_command     = new byte[1];
            zero_yaw_command[0]         = AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_YAW;

            if ( enable_logging ) {
                Log.i("navx_ftc", "Beginning communication with navX-Model sensor " + sensor.getDeviceName() + " at I2C address " + sensor.getI2cAddress());
                Log.i("navx_ftc", "Firmware version:  " + sensor.getFirmwareVersion());
            }

            setConnected(false);

            long ms_per_loop = (1000/update_rate_hz) - 1;
            if (ms_per_loop < 5) {
                ms_per_loop = 5; // Dont allow the loop to interrupt more than once every 5 ms (1000 ms/200hz)
            }

            while ( keep_running ) {
                long ms_at_loop_start = SystemClock.elapsedRealtime();
                try {
                    if ( !is_connected ) {
                        this.last_second_hertz = 0;
                        this.last_valid_sensor_timestamp = 0;
                        this.byte_count = 0;
                        this.update_count = 0;
                        this.hertz_counter = 0;
                        this.duplicate_sensor_data_count = 0;
                        TimestampedData board_data = sensor.readTimeStamped(NavxMicroNavigationSensor.Register.WHOAMI, DIM_MAX_I2C_READ_LEN);
                        if ((board_data != null) && (board_data.data != null)) {
                            if (decodeNavxBoardData(board_data.data, NAVX_REGISTER_FIRST, board_data.data.length)) {
                                setConnected(true);

                                /* To handle the case where the device is reset, reconfigure the */
                                /* update rate whenever reconecting to the device.               */
                                // TODO:  How do we detect a timeout here?
                                sensor.write8(NavxMicroNavigationSensor.Register.UPDATE_RATE_HZ, update_rate_command[0]);

                                /* Re-read the board data after the update rate is modified. */
                                board_data = sensor.readTimeStamped(NavxMicroNavigationSensor.Register.WHOAMI, DIM_MAX_I2C_READ_LEN);
                                if ((board_data == null) || (board_data.data == null) ||
                                    !decodeNavxBoardData(board_data.data, NAVX_REGISTER_FIRST, board_data.data.length)) {
                                    setConnected(false);
                                }
                            }
                        }
                    } else {
                        /* If connected, read sensor data and optionally zero yaw if requested */
                        synchronized(reset_yaw_critical_section) {
                            if (request_zero_yaw) {

                                sensor.write8(NavxMicroNavigationSensor.Register.INTEGRATION_CTL, zero_yaw_command[0]);
                                /* After zeroing the yaw, wait one sample time to ensure that */
                                /* a new yaw value (which has been "zeroed") is ready to read. */
                                // TODO:  How do we detect a timeout here?
                                Thread.sleep(1000 / update_rate_hz);
                                request_zero_yaw = false;
                            }
                        }

                        /* Read Processed Data (kProcessedData or kAll) */

                        if ((data_type == DeviceDataType.kProcessedData) ||
                                (data_type == DeviceDataType.kAll)) {
                            TimestampedData processed_data = sensor.readTimeStamped(NavxMicroNavigationSensor.Register.SENSOR_STATUS_L, DIM_MAX_I2C_READ_LEN);
                            if (( processed_data != null) && (processed_data.data != null)) {
                                // TODO:  Verify that the following function will fail if
                                // device is disconnected.  It is assumed that in this case,
                                // the returned data will be all zeros, which will cause the
                                // decode to fail.
                                if (!decodeNavxProcessedData(processed_data.data,
                                        NAVX_REGISTER_PROC_FIRST, processed_data.data.length)) {
                                    setConnected(false);
                                } else {
                                    if (curr_sensor_timestamp != last_valid_sensor_timestamp) {
                                        addToByteCount(processed_data.data.length);
                                        incrementUpdateCount();
                                        for (int i = 0; i < callbacks.length; i++) {
                                            IDataArrivalSubscriber callback = callbacks[i];
                                            if (callback != null) {
                                                long sys_timestamp_ms = processed_data.nanoTime / 1000000;
                                                callback.timestampedDataReceived(sys_timestamp_ms,
                                                        curr_sensor_timestamp,
                                                        DeviceDataType.kProcessedData);
                                            }
                                        }
                                    } else {
                                        duplicate_sensor_data_count++;
                                    }

                                    if ((curr_sensor_timestamp % 1000) < (last_valid_sensor_timestamp % 1000)) {
                                        /* Second roll over.  Start the Hertz accumulator */
                                        last_second_hertz = hertz_counter;
                                        hertz_counter = 1;
                                    } else {
                                        hertz_counter++;
                                    }

                                    last_valid_sensor_timestamp = curr_sensor_timestamp;
                                }
                            }
                        }

                        /* Read Quaternion/Raw Data (kQuatAndRawData or kAll) */

                        if ((data_type == DeviceDataType.kQuatAndRawData) ||
                                (data_type == DeviceDataType.kAll)) {
                            TimestampedData raw_data = sensor.readTimeStamped(NavxMicroNavigationSensor.Register.QUAT_W_L, DIM_MAX_I2C_READ_LEN);

                            if ( (raw_data != null) && (raw_data.data != null)) {
                                // TODO:  Verify that the following function will fail if
                                // device is disconnected.  It is assumed that in this case,
                                // the returned data will be all zeros, which will cause the
                                // decode to fail.
                                if ( !decodeNavxQuatAndRawData(raw_data.data,
                                        NAVX_REGISTER_RAW_FIRST, raw_data.data.length) ) {
                                    setConnected(false);
                                } else {
                                    addToByteCount(raw_data.data.length);
                                    incrementUpdateCount();
                                    for ( int i = 0; i < callbacks.length; i++ ) {
                                        IDataArrivalSubscriber callback = callbacks[i];
                                        if (callback != null) {
                                            callback.untimestampedDataReceived(raw_data.nanoTime,
                                                    DeviceDataType.kQuatAndRawData );
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (keep_running) {
                        try {
                            long ms_now = SystemClock.elapsedRealtime();
                            long ms_last_loop_iteration_duration = ms_now - ms_at_loop_start;
                            long sleep_ms = ms_per_loop - ms_last_loop_iteration_duration;
                            if (sleep_ms > 0) {
                                // Typically, block until it's again time to read new device data;
                                // however if certain events (yaw reset request, or thread shutdown)
                                // occur, immediate unblocking should occur to minimize response time.
                                synchronized (io_thread_event) {
                                    io_thread_event.wait(sleep_ms);
                                }
                            }
                        } catch (InterruptedException e) {
                            // This handles the case that the thread is being shutdown abruptly.
                            keep_running = false;
                            e.printStackTrace();
                        }
                    }
                } catch (Exception ex) {
                    ex.printStackTrace();
                }
            }

            /* IO thread is shutting down; indicate device is no longer connected. */
            setConnected(false);

            if ( enable_logging ) {
                Log.i("navx_ftc", "Closing I2C device.");
            }
        }

        boolean decodeNavxBoardData(byte[] curr_data, int first_address, int len) {
            final int I2C_NAVX_DEVICE_TYPE = 50;
            boolean valid_data;
            if ((curr_data == null) ||
                    (len < (IMURegisters.NAVX_REG_CAPABILITY_FLAGS_H - first_address))) {
                return false;
            }
            if (curr_data[IMURegisters.NAVX_REG_WHOAMI - first_address] == I2C_NAVX_DEVICE_TYPE){
                valid_data = true;
                board_id.hw_rev = curr_data[IMURegisters.NAVX_REG_HW_REV - first_address];
                board_id.fw_ver_major = curr_data[IMURegisters.NAVX_REG_FW_VER_MAJOR - first_address];
                board_id.fw_ver_minor = curr_data[IMURegisters.NAVX_REG_FW_VER_MINOR - first_address];
                board_id.type = curr_data[IMURegisters.NAVX_REG_WHOAMI - first_address];

                board_state.gyro_fsr_dps = AHRSProtocol.decodeBinaryUint16(curr_data, IMURegisters.NAVX_REG_GYRO_FSR_DPS_L - first_address);
                board_state.accel_fsr_g = (short) curr_data[IMURegisters.NAVX_REG_ACCEL_FSR_G - first_address];
                board_state.update_rate_hz = curr_data[IMURegisters.NAVX_REG_UPDATE_RATE_HZ - first_address];
                board_state.capability_flags = AHRSProtocol.decodeBinaryUint16(curr_data, IMURegisters.NAVX_REG_CAPABILITY_FLAGS_L - first_address);
                board_state.op_status = curr_data[IMURegisters.NAVX_REG_OP_STATUS - first_address];
                board_state.selftest_status = curr_data[IMURegisters.NAVX_REG_SELFTEST_STATUS - first_address];
                board_state.cal_status = curr_data[IMURegisters.NAVX_REG_CAL_STATUS - first_address];
            } else {
                valid_data = false;
            }
            return valid_data;
        }

        boolean doesDataAppearValid( byte[] curr_data ) {
            boolean data_valid = false;
            boolean all_zeros = true;
            boolean all_ones = true;
            for ( int i = 0; i < curr_data.length; i++ ) {
                if ( curr_data[i] != (byte)0 ) {
                    all_zeros = false;
                }
                if ( curr_data[i] != (byte)0xFF) {
                    all_ones = false;
                }
                if ( !all_zeros && !all_ones ) {
                    data_valid = true;
                    break;
                }
            }
            return data_valid;
        }

        boolean decodeNavxProcessedData(byte[] curr_data, int first_address, int len) {
            long timestamp_low, timestamp_high;

            if ((curr_data == null) ||
                    (len < (IMURegisters.NAVX_REG_LINEAR_ACC_Z_H - first_address))) {
                return false;
            }
            boolean data_valid = doesDataAppearValid(curr_data);
            if ( !data_valid ) {
                Arrays.fill(curr_data, (byte)0);
            }
            curr_sensor_timestamp = (long)AHRSProtocol.decodeBinaryUint32(curr_data, IMURegisters.NAVX_REG_TIMESTAMP_L_L - first_address);
            ahrspos_update.sensor_status = curr_data[IMURegisters.NAVX_REG_SENSOR_STATUS_L - first_address];
            /* Update calibration status from the "shadow" in the upper 8-bits of sensor status. */
            ahrspos_update.cal_status = curr_data[IMURegisters.NAVX_REG_SENSOR_STATUS_H - first_address];
            ahrspos_update.yaw = AHRSProtocol.decodeProtocolSignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_YAW_L - first_address);
            ahrspos_update.pitch = AHRSProtocol.decodeProtocolSignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_PITCH_L - first_address);
            ahrspos_update.roll = AHRSProtocol.decodeProtocolSignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_ROLL_L - first_address);
            ahrspos_update.compass_heading = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_HEADING_L - first_address);
            ahrspos_update.fused_heading = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_FUSED_HEADING_L - first_address);
            ahrspos_update.altitude = AHRSProtocol.decodeProtocol1616Float(curr_data, IMURegisters.NAVX_REG_ALTITUDE_I_L - first_address);
            ahrspos_update.linear_accel_x = AHRSProtocol.decodeProtocolSignedThousandthsFloat(curr_data, IMURegisters.NAVX_REG_LINEAR_ACC_X_L - first_address);
            ahrspos_update.linear_accel_y = AHRSProtocol.decodeProtocolSignedThousandthsFloat(curr_data, IMURegisters.NAVX_REG_LINEAR_ACC_Y_L - first_address);
            ahrspos_update.linear_accel_z = AHRSProtocol.decodeProtocolSignedThousandthsFloat(curr_data, IMURegisters.NAVX_REG_LINEAR_ACC_Z_L - first_address);

            return data_valid;
        }

        boolean decodeNavxQuatAndRawData(byte[] curr_data, int first_address, int len) {
            if ((curr_data == null) ||
                    (len < (IMURegisters.NAVX_REG_MAG_Y_H - first_address))) {
                return false;
            }
            boolean data_valid = doesDataAppearValid(curr_data);
            if ( !data_valid ) {
                Arrays.fill(curr_data, (byte)0);
            }
            ahrspos_update.quat_w   = AHRSProtocol.decodeBinaryInt16(curr_data, IMURegisters.NAVX_REG_QUAT_W_L-first_address);
            ahrspos_update.quat_x   = AHRSProtocol.decodeBinaryInt16(curr_data, IMURegisters.NAVX_REG_QUAT_X_L-first_address);
            ahrspos_update.quat_y   = AHRSProtocol.decodeBinaryInt16(curr_data, IMURegisters.NAVX_REG_QUAT_Y_L-first_address);
            ahrspos_update.quat_z   = AHRSProtocol.decodeBinaryInt16(curr_data, IMURegisters.NAVX_REG_QUAT_Z_L-first_address);

            ahrspos_update.mpu_temp = AHRSProtocol.decodeProtocolSignedHundredthsFloat(curr_data, IMURegisters.NAVX_REG_MPU_TEMP_C_L - first_address);

            raw_data_update.gyro_x  = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_GYRO_X_L-first_address);
            raw_data_update.gyro_y  = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_GYRO_Y_L-first_address);
            raw_data_update.gyro_z  = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_GYRO_Z_L-first_address);
            raw_data_update.accel_x = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_ACC_X_L-first_address);
            raw_data_update.accel_y = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_ACC_Y_L-first_address);
            raw_data_update.accel_z = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_ACC_Z_L-first_address);
            raw_data_update.mag_x   = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_MAG_X_L-first_address);
            raw_data_update.mag_y   = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_MAG_Y_L-first_address);
            /* Unfortunately, the 26-byte I2C Transfer limit means we can't transfer the Z-axis magnetometer data.  */
            /* This magnetomer axis typically isn't used, so it's likely not going to be missed.                    */
            //raw_data_update.mag_z   = AHRSProtocol.decodeBinaryInt16(curr_data,  IMURegisters.NAVX_REG_MAG_Z_L-first_address);

            return data_valid;
        }

    }
}
