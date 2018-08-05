package org.firstinspires.ftc.teamcode;

import com.acmerobotics.splinelib.drive.Drive;
import com.acmerobotics.splinelib.drive.TrackWidthCalibrationOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@Autonomous
public class MyTrackWidthCalibrationOpMode extends TrackWidthCalibrationOpMode {
    @Override
    protected Drive initDrive() {
        return new MyMecanumDrive(hardwareMap);
    }

    @Override
    protected BNO055IMU initIMU() {
        LynxModule frontHub = hardwareMap.get(LynxModule.class, "frontHub");
        I2cDeviceSynch imuI2cDevice = LynxOptimizedI2cFactory.createLynxI2cDeviceSynch(frontHub, 0);
        imuI2cDevice.setUserConfiguredName("imu");
        BNO055IMU imu = new LynxEmbeddedIMU(imuI2cDevice);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // taken from https://ftcforum.usfirst.org/forum/ftc-technology/53812-mounting-the-revhub-vertically?p=56587#post56587
        // testing suggests that an axis remap is more accurate then simply changing the axis read
        // we hypothesize that this helps properly configure the internal sensor fusion/Kalman filtering
        try {
            // axis remap
            byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
            byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

            //Need to be in CONFIG mode to write to registers
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            Thread.sleep(100); //Changing modes requires a delay before doing anything else

            //Write to the AXIS_MAP_CONFIG register
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);

            //Write to the AXIS_MAP_SIGN register
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

            //Need to change back into the IMU mode to use the gyro
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

            Thread.sleep(100); //Changing modes again requires a delay
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        return imu;
    }
}
