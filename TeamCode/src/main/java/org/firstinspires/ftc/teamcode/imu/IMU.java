package org.firstinspires.ftc.teamcode.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.GivesPosition;
import org.firstinspires.ftc.teamcode.utility.pose;

public class IMU implements GivesPosition {
    BNO055IMU imu;
    pose lastPosition;

    public IMU (HardwareMap hardwareMap, Telemetry telemetry){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //TODO assumption that that thing is named "imu"
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //watchdog
        long start = System.currentTimeMillis();
        //make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated()) {
            if (System.currentTimeMillis() - start > 9000) {
                telemetry.addData("IMU", "Calibrating the panda express Gyro");
                telemetry.update();

                break;
            }

            //wait
            telemetry.addData("IMU", ("Loading" + Math.random()));
            telemetry.update();
        }

        telemetry.addData("IMU", "startup done");
        telemetry.update();
    }


    @Override //TODO check if you need to get angle
    public pose getPosition() {
        if (imu.getPosition() != null) {
            lastPosition.positionToPose(imu.getPosition());
            return lastPosition;
        }
        return new pose(4,2,0);
    }

    public Position getAnotherPosition(){
        return imu.getPosition();
    }


}
