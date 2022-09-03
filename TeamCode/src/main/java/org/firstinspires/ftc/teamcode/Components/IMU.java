package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.BasicRobot.op;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class IMU {
    private BNO055IMU imu;
    public IMU() {
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.startAccelerationIntegration(new Position(),new Velocity(),20);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!op.isStopRequested() && !imu.isGyroCalibrated()) {
            op.sleep(50);
            op.idle();
        }
    }

    public double updateAngle(){
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        if(angle<0){
            angle+=2*PI;
        }
        return angle;
    }
    public AngularVelocity getAngularVelocity(){
        return imu.getAngularVelocity();
    }

}
