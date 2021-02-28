package org.firstinspires.ftc.robot_utilities;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RotationController {

    private Orientation lastAngles;
    private double currentHeading = 0;
    private PIDController pidRotate;
    public BNO055IMU imu;

    public RotationController(BNO055IMU imu) {
        pidRotate = new PIDController(Vals.rotate_kp, Vals.rotate_ki, Vals.rotate_kd);
        pidRotate.setTolerance(Vals.rotate_tolerance);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        this.imu = imu;
        imu.initialize(parameters);
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = 0;
        pidRotate.reset();
    }

    public double updateHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle; //current robot needs z axis tho this may change later

        if(deltaAngle < -180) {
            deltaAngle += 360;
        } else if(deltaAngle > 180) {
            deltaAngle -= 360;
        }

        currentHeading += deltaAngle;
        lastAngles = angles;

        return currentHeading;
    }

    public double rotate(double degrees) {
        if(Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        double power = pidRotate.calculate(updateHeading(), degrees);
        return power;

    }
}
