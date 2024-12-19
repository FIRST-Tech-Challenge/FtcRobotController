
/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;


@I2cDeviceType
@DeviceProperties(
        name = "goBILDA® Pinpoint Odometry Computer",
        xmlTag = "goBILDAPinpoint",
        description ="goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
)

public class GoBildaPinpointDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {

    private int deviceStatus   = 0;
    private int loopTime       = 0;
    private int xEncoderValue  = 0;
    private int yEncoderValue  = 0;
    private float xPosition    = 0;
    private float yPosition    = 0;
    private float hOrientation = 0;
    private float xVelocity    = 0;
    private float yVelocity    = 0;
    private float hVelocity    = 0;

    private static final float goBILDA_SWINGARM_POD = 13.26291192f; //ticks-per-mm for the goBILDA Swingarm Pod
    private static final float goBILDA_4_BAR_POD    = 19.89436789f; //ticks-per-mm for the goBILDA 4-Bar Pod

    //i2c address of the device
    public static final byte DEFAULT_ADDRESS = 0x31;

    public GoBildaPinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
    }


    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        ((LynxI2cDeviceSynch)(deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        return true;
    }

    @Override
    public String getDeviceName() {
        return "goBILDA® Pinpoint Odometry Computer";
    }


    //Register map of the i2c device
    private enum Register {
        DEVICE_ID       (1),
        DEVICE_VERSION  (2),
        DEVICE_STATUS   (3),
        DEVICE_CONTROL  (4),
        LOOP_TIME       (5),
        X_ENCODER_VALUE (6),
        Y_ENCODER_VALUE (7),
        X_POSITION      (8),
        Y_POSITION      (9),
        H_ORIENTATION   (10),
        X_VELOCITY      (11),
        Y_VELOCITY      (12),
        H_VELOCITY      (13),
        MM_PER_TICK     (14),
        X_POD_OFFSET    (15),
        Y_POD_OFFSET    (16),
        YAW_SCALAR      (17),
        BULK_READ       (18);

        private final int bVal;

        Register(int bVal){
            this.bVal = bVal;
        }
    }

    //Device Status enum that captures the current fault condition of the device
    public enum DeviceStatus{
        NOT_READY                (0),
        READY                    (1),
        CALIBRATING              (1 << 1),
        FAULT_X_POD_NOT_DETECTED (1 << 2),
        FAULT_Y_POD_NOT_DETECTED (1 << 3),
        FAULT_NO_PODS_DETECTED   (1 << 2 | 1 << 3),
        FAULT_IMU_RUNAWAY        (1 << 4);

        private final int status;

        DeviceStatus(int status){
            this.status = status;
        }
    }

    //enum that captures the direction the encoders are set to
    public enum EncoderDirection{
        FORWARD,
        REVERSED;
    }

    //enum that captures the kind of goBILDA odometry pods, if goBILDA pods are used
    public enum GoBildaOdometryPods {
        goBILDA_SWINGARM_POD,
        goBILDA_4_BAR_POD;
    }
    //enum that captures a limited scope of read data. More options may be added in future update
    public enum readData {
        ONLY_UPDATE_HEADING,
    }


    /** Writes an int to the i2c device
     @param reg the register to write the int to
     @param i the integer to write to the register
     */
    private void writeInt(final Register reg, int i){
        deviceClient.write(reg.bVal, TypeConversion.intToByteArray(i,ByteOrder.LITTLE_ENDIAN));
    }

    /**
     * Reads an int from a register of the i2c device
     * @param reg the register to read from
     * @return returns an int that contains the value stored in the read register
     */
    private int readInt(Register reg){
        return byteArrayToInt(deviceClient.read(reg.bVal,4), ByteOrder.LITTLE_ENDIAN);
    }

    /**
     * Converts a byte array to a float value
     * @param byteArray byte array to transform
     * @param byteOrder order of byte array to convert
     * @return the float value stored by the byte array
     */
    private float byteArrayToFloat(byte[] byteArray, ByteOrder byteOrder){
        return ByteBuffer.wrap(byteArray).order(byteOrder).getFloat();
    }
    /**
     * Reads a float from a register
     * @param reg the register to read
     * @return the float value stored in that register
     */

    private float readFloat(Register reg){
        return byteArrayToFloat(deviceClient.read(reg.bVal,4),ByteOrder.LITTLE_ENDIAN);
    }


    /**
     * Converts a float to a byte array
     * @param value the float array to convert
     * @return the byte array converted from the float
     */
    private byte [] floatToByteArray (float value, ByteOrder byteOrder) {
        return ByteBuffer.allocate(4).order(byteOrder).putFloat(value).array();
    }

    /**
     * Writes a byte array to a register on the i2c device
     * @param reg the register to write to
     * @param bytes the byte array to write
     */
    private void writeByteArray (Register reg, byte[] bytes){
        deviceClient.write(reg.bVal,bytes);
    }

    /**
     * Writes a float to a register on the i2c device
     * @param reg the register to write to
     * @param f the float to write
     */
    private void writeFloat (Register reg, float f){
        byte[] bytes = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(f).array();
        deviceClient.write(reg.bVal,bytes);
    }

    /**
     * Looks up the DeviceStatus enum corresponding with an int value
     * @param s int to lookup
     * @return the Odometry Computer state
     */
    private DeviceStatus lookupStatus (int s){
        if ((s & DeviceStatus.CALIBRATING.status) != 0){
            return DeviceStatus.CALIBRATING;
        }
        boolean xPodDetected = (s & DeviceStatus.FAULT_X_POD_NOT_DETECTED.status) == 0;
        boolean yPodDetected = (s & DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status) == 0;

        if(!xPodDetected  && !yPodDetected){
            return DeviceStatus.FAULT_NO_PODS_DETECTED;
        }
        if (!xPodDetected){
            return DeviceStatus.FAULT_X_POD_NOT_DETECTED;
        }
        if (!yPodDetected){
            return DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
        }
        if ((s & DeviceStatus.FAULT_IMU_RUNAWAY.status) != 0){
            return DeviceStatus.FAULT_IMU_RUNAWAY;
        }
        if ((s & DeviceStatus.READY.status) != 0){
            return DeviceStatus.READY;
        }
        else {
            return DeviceStatus.NOT_READY;
        }
    }

    /**
     * Call this once per loop to read new data from the Odometry Computer. Data will only update once this is called.
     */
    public void update(){
        byte[] bArr   = deviceClient.read(Register.BULK_READ.bVal, 40);
        deviceStatus  = byteArrayToInt(Arrays.copyOfRange  (bArr, 0, 4),  ByteOrder.LITTLE_ENDIAN);
        loopTime      = byteArrayToInt(Arrays.copyOfRange  (bArr, 4, 8),  ByteOrder.LITTLE_ENDIAN);
        xEncoderValue = byteArrayToInt(Arrays.copyOfRange  (bArr, 8, 12), ByteOrder.LITTLE_ENDIAN);
        yEncoderValue = byteArrayToInt(Arrays.copyOfRange  (bArr, 12,16), ByteOrder.LITTLE_ENDIAN);
        xPosition     = byteArrayToFloat(Arrays.copyOfRange(bArr, 16,20), ByteOrder.LITTLE_ENDIAN);
        yPosition     = byteArrayToFloat(Arrays.copyOfRange(bArr, 20,24), ByteOrder.LITTLE_ENDIAN);
        hOrientation  = byteArrayToFloat(Arrays.copyOfRange(bArr, 24,28), ByteOrder.LITTLE_ENDIAN);
        xVelocity     = byteArrayToFloat(Arrays.copyOfRange(bArr, 28,32), ByteOrder.LITTLE_ENDIAN);
        yVelocity     = byteArrayToFloat(Arrays.copyOfRange(bArr, 32,36), ByteOrder.LITTLE_ENDIAN);
        hVelocity     = byteArrayToFloat(Arrays.copyOfRange(bArr, 36,40), ByteOrder.LITTLE_ENDIAN);
    }

    /**
     * Call this once per loop to read new data from the Odometry Computer. This is an override of the update() function
     * which allows a narrower range of data to be read from the device for faster read times. Currently ONLY_UPDATE_HEADING
     * is supported.
     * @param data GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING
     */
    public void update(readData data) {
        if (data == readData.ONLY_UPDATE_HEADING) {
            hOrientation = byteArrayToFloat(deviceClient.read(Register.H_ORIENTATION.bVal, 4), ByteOrder.LITTLE_ENDIAN);
        }
    }

    /**
     * Sets the odometry pod positions relative to the point that the odometry computer tracks around.<br><br>
     * The most common tracking position is the center of the robot. <br> <br>
     * The X pod offset refers to how far sideways (in mm) from the tracking point the X (forward) odometry pod is. Left of the center is a positive number, right of center is a negative number. <br>
     * the Y pod offset refers to how far forwards (in mm) from the tracking point the Y (strafe) odometry pod is. forward of center is a positive number, backwards is a negative number.<br>
     * @param xOffset how sideways from the center of the robot is the X (forward) pod? Left increases
     * @param yOffset how far forward from the center of the robot is the Y (Strafe) pod? forward increases
     */
    public void setOffsets(double xOffset, double yOffset){
        writeFloat(Register.X_POD_OFFSET, (float) xOffset);
        writeFloat(Register.Y_POD_OFFSET, (float) yOffset);
    }

    /**
     * Recalibrates the Odometry Computer's internal IMU. <br><br>
     * <strong> Robot MUST be stationary </strong> <br><br>
     * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
     */
    public void recalibrateIMU(){writeInt(Register.DEVICE_CONTROL,1<<0);}

    /**
     * Resets the current position to 0,0,0 and recalibrates the Odometry Computer's internal IMU. <br><br>
     * <strong> Robot MUST be stationary </strong> <br><br>
     * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
     */
    public void resetPosAndIMU(){writeInt(Register.DEVICE_CONTROL,1<<1);}

    /**
     * Can reverse the direction of each encoder.
     * @param xEncoder FORWARD or REVERSED, X (forward) pod should increase when the robot is moving forward
     * @param yEncoder FORWARD or REVERSED, Y (strafe) pod should increase when the robot is moving left
     */
    public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder){
        if (xEncoder == EncoderDirection.FORWARD){
            writeInt(Register.DEVICE_CONTROL,1<<5);
        }
        if (xEncoder == EncoderDirection.REVERSED) {
            writeInt(Register.DEVICE_CONTROL,1<<4);
        }

        if (yEncoder == EncoderDirection.FORWARD){
            writeInt(Register.DEVICE_CONTROL,1<<3);
        }
        if (yEncoder == EncoderDirection.REVERSED){
            writeInt(Register.DEVICE_CONTROL,1<<2);
        }
    }

    /**
     * If you're using goBILDA odometry pods, the ticks-per-mm values are stored here for easy access.<br><br>
     * @param pods goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD
     */
    public void setEncoderResolution(GoBildaOdometryPods pods){
        if (pods == GoBildaOdometryPods.goBILDA_SWINGARM_POD) {
            writeByteArray(Register.MM_PER_TICK, (floatToByteArray(goBILDA_SWINGARM_POD, ByteOrder.LITTLE_ENDIAN)));
        }
        if (pods == GoBildaOdometryPods.goBILDA_4_BAR_POD){
            writeByteArray(Register.MM_PER_TICK,(floatToByteArray(goBILDA_4_BAR_POD, ByteOrder.LITTLE_ENDIAN)));
        }
    }

    /**
     * Sets the encoder resolution in ticks per mm of the odometry pods. <br>
     * You can find this number by dividing the counts-per-revolution of your encoder by the circumference of the wheel.
     * @param ticks_per_mm should be somewhere between 10 ticks/mm and 100 ticks/mm a goBILDA Swingarm pod is ~13.26291192
     */
    public void setEncoderResolution(double ticks_per_mm){
        writeByteArray(Register.MM_PER_TICK,(floatToByteArray((float) ticks_per_mm,ByteOrder.LITTLE_ENDIAN)));
    }

    /**
     * Tuning this value should be unnecessary.<br>
     * The goBILDA Odometry Computer has a per-device tuned yaw offset already applied when you receive it.<br><br>
     * This is a scalar that is applied to the gyro's yaw value. Increasing it will mean it will report more than one degree for every degree the sensor fusion algorithm measures. <br><br>
     * You can tune this variable by rotating the robot a large amount (10 full turns is a good starting place) and comparing the amount that the robot rotated to the amount measured.
     * Rotating the robot exactly 10 times should measure 3600°. If it measures more or less, divide moved amount by the measured amount and apply that value to the Yaw Offset.<br><br>
     * If you find that to get an accurate heading number you need to apply a scalar of more than 1.05, or less than 0.95, your device may be bad. Please reach out to tech@gobilda.com
     * @param yawOffset A scalar for the robot's heading.
     */
    public void setYawScalar(double yawOffset){
        writeByteArray(Register.YAW_SCALAR,(floatToByteArray((float) yawOffset, ByteOrder.LITTLE_ENDIAN)));
    }

    /**
     * Send a position that the Pinpoint should use to track your robot relative to. You can use this to
     * update the estimated position of your robot with new external sensor data, or to run a robot
     * in field coordinates. <br><br>
     * This overrides the current position. <br><br>
     * <strong>Using this feature to track your robot's position in field coordinates:</strong> <br>
     * When you start your code, send a Pose2D that describes the starting position on the field of your robot. <br>
     * Say you're on the red alliance, your robot is against the wall and closer to the audience side,
     * and the front of your robot is pointing towards the center of the field.
     * You can send a setPosition with something like -600mm x, -1200mm Y, and 90 degrees. The pinpoint would then always
     * keep track of how far away from the center of the field you are. <br><br>
     * <strong>Using this feature to update your position with additional sensors: </strong><br>
     * Some robots have a secondary way to locate their robot on the field. This is commonly
     * Apriltag localization in FTC, but it can also be something like a distance sensor.
     * Often these external sensors are absolute (meaning they measure something about the field)
     * so their data is very accurate. But they can be slower to read, or you may need to be in a very specific
     * position on the field to use them. In that case, spend most of your time relying on the Pinpoint
     * to determine your location. Then when you pull a new position from your secondary sensor,
     * send a setPosition command with the new position. The Pinpoint will then track your movement
     * relative to that new, more accurate position.
     * @param pos a Pose2D describing the robot's new position.
     */
    public Pose2D setPosition(Pose2D pos){
        writeByteArray(Register.X_POSITION,(floatToByteArray((float) pos.getX(DistanceUnit.MM), ByteOrder.LITTLE_ENDIAN)));
        writeByteArray(Register.Y_POSITION,(floatToByteArray((float) pos.getY(DistanceUnit.MM),ByteOrder.LITTLE_ENDIAN)));
        writeByteArray(Register.H_ORIENTATION,(floatToByteArray((float) pos.getHeading(AngleUnit.RADIANS),ByteOrder.LITTLE_ENDIAN)));
        return pos;
    }

    /**
     * Checks the deviceID of the Odometry Computer. Should return 1.
     * @return 1 if device is functional.
     */
    public int getDeviceID(){return readInt(Register.DEVICE_ID);}

    /**
     * @return the firmware version of the Odometry Computer
     */
    public int getDeviceVersion(){return readInt(Register.DEVICE_VERSION); }

    public float getYawScalar(){return readFloat(Register.YAW_SCALAR); }

    /**
     * Device Status stores any faults the Odometry Computer may be experiencing. These faults include:
     * @return one of the following states:<br>
     * NOT_READY - The device is currently powering up. And has not initialized yet. RED LED<br>
     * READY - The device is currently functioning as normal. GREEN LED<br>
     * CALIBRATING - The device is currently recalibrating the gyro. RED LED<br>
     * FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in. PURPLE LED <br>
     * FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in. BLUE LED <br>
     * FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in. ORANGE LED <br>
     */
    public DeviceStatus getDeviceStatus(){return lookupStatus(deviceStatus); }

    /**
     * Checks the Odometry Computer's most recent loop time.<br><br>
     * If values less than 500, or more than 1100 are commonly seen here, there may be something wrong with your device. Please reach out to tech@gobilda.com
     * @return loop time in microseconds (1/1,000,000 seconds)
     */
    public int getLoopTime(){return loopTime; }

    /**
     * Checks the Odometry Computer's most recent loop frequency.<br><br>
     * If values less than 900, or more than 2000 are commonly seen here, there may be something wrong with your device. Please reach out to tech@gobilda.com
     * @return Pinpoint Frequency in Hz (loops per second),
     */
    public double getFrequency(){
        if (loopTime != 0){
            return 1000000.0/loopTime;
        }
        else {
            return 0;
        }
    }

    /**
     * @return the raw value of the X (forward) encoder in ticks
     */
    public int getEncoderX(){return xEncoderValue; }

    /**
     * @return the raw value of the Y (strafe) encoder in ticks
     */
    public int getEncoderY(){return yEncoderValue; }

    /**
     * @return the estimated X (forward) position of the robot in mm
     */
    public double getPosX(){return xPosition; }

    /**
     * @return the estimated Y (Strafe) position of the robot in mm
     */
    public double getPosY(){return yPosition; }

    /**
     * @return the estimated H (heading) position of the robot in Radians
     */
    public double getHeading(){return hOrientation;}

    /**
     * @return the estimated X (forward) velocity of the robot in mm/sec
     */
    public double getVelX(){return xVelocity; }

    /**
     * @return the estimated Y (strafe) velocity of the robot in mm/sec
     */
    public double getVelY(){return yVelocity; }

    /**
     * @return the estimated H (heading) velocity of the robot in radians/sec
     */
    public double getHeadingVelocity(){return hVelocity; }

    /**
     * <strong> This uses its own I2C read, avoid calling this every loop. </strong>
     * @return the user-set offset for the X (forward) pod
     */
    public float getXOffset(){return readFloat(Register.X_POD_OFFSET);}

    /**
     * <strong> This uses its own I2C read, avoid calling this every loop. </strong>
     * @return the user-set offset for the Y (strafe) pod
     */
    public float getYOffset(){return readFloat(Register.Y_POD_OFFSET);}

    /**
     * @return a Pose2D containing the estimated position of the robot
     */
    public Pose2D getPosition(){
        return new Pose2D(DistanceUnit.MM,
                xPosition,
                yPosition,
                AngleUnit.RADIANS,
                hOrientation);
    }



    /**
     * @return a Pose2D containing the estimated velocity of the robot, velocity is unit per second
     */
    public Pose2D getVelocity(){
        return new Pose2D(DistanceUnit.MM,
                xVelocity,
                yVelocity,
                AngleUnit.RADIANS,
                hVelocity);
    }



}