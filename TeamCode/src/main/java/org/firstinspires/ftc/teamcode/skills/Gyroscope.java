package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV1;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Gyroscope {
    BNO055IMU imu;
    Telemetry telemetry;
    private int desiredHeading = 0;
    private boolean calib = false;
    public static String CALIB_FILE = "BNO055IMUCalibration.json";


    public void init(HardwareMap ahwMap, Telemetry t, boolean calibrate){
        telemetry = t;
        this.calib = calibrate;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = this.calib;
        parameters.loggingTag          = "IMU";

        if (!this.calib) {
            parameters.calibrationDataFile = CALIB_FILE;
        }
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = (BNO055IMU) ahwMap.get("imu");
        if (imu != null){
            imu.initialize(parameters);
            telemetry.addData("Info", "Gyroscope initialized");
        }
        else{
            telemetry.addData("Error", "Gyroscope failed");
        }

    }

    public void calibrate(){
        this.calib = true;
        while (calib) {
            if (imu.isGyroCalibrated() && calib) {
                BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
                String filename = CALIB_FILE;
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                telemetry.log().add("saved to '%s'", filename);
                calib = false;
            }else{
                telemetry.addLine("Calibrating...");
                telemetry.update();
            }
        }
        if (!calib) {
            telemetry.addLine("Calibration complete. Restart the opmode to initialize the gyroscope");
            telemetry.update();
        }
    }


    public double getHeading()  {
        Orientation orient = imu.getAngularOrientation();
        double angle = orient.firstAngle;

        return angle;
    }

    public int getDesiredHeading() {
        return desiredHeading;
    }

    public void setDesiredHeading(int desired) {
        desiredHeading = desired;
    }
}
