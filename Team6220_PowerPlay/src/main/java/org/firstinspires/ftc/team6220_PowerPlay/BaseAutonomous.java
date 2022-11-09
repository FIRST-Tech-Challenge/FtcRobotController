package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public abstract class BaseAutonomous extends BaseOpMode {

    /**
     * this method will allow the robot to drive straight in a specified direction using the IMU given a specified heading and distance
     * @param heading 360-degree direction robot should move (front is 0)
     * @param targetDistance distance robot should move in inches
     */
    public void driveOmniInches(int heading, double targetDistance) {
        Position position = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0);
        Velocity velocity = new Velocity(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0);

        imu.startAccelerationIntegration(position, velocity, 10);

        boolean distanceReached = false;

        double distanceTraveled;
        double angleDeviation;
        double motorPower;
        double turningPower;

        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (!distanceReached && opModeIsActive()) {
            angleDeviation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle;

            turningPower = angleDeviation / 50;

            // todo - fix distance
            distanceTraveled = Math.sqrt(Math.pow(imu.getPosition().x, 2) + Math.pow(imu.getPosition().y, 2));

            // todo - max linear acceleration where encoders don't slip, then 0, then max linear deceleration where encoders don't slip
            motorPower = Math.max(Math.sqrt(Math.sin(distanceTraveled / targetDistance * Math.PI)) * 0.25, 0.1);

            if (distanceTraveled > targetDistance) {
                motorFL.setPower(0.0);
                motorFR.setPower(0.0);
                motorBL.setPower(0.0);
                motorBR.setPower(0.0);

                distanceReached = true;

                imu.stopAccelerationIntegration();
            }

            // todo - make work for any heading
            if (heading == 0) {
                motorFL.setPower(motorPower + turningPower);
                motorFR.setPower(motorPower - turningPower);
                motorBL.setPower(motorPower + turningPower);
                motorBR.setPower(motorPower - turningPower);
            } else if (heading == 90) {
                motorFL.setPower(-motorPower + turningPower);
                motorFR.setPower(-motorPower - turningPower);
                motorBL.setPower(-motorPower + turningPower);
                motorBR.setPower(-motorPower - turningPower);
            } else if (heading == 180) {
                motorFL.setPower(-motorPower + turningPower);
                motorFR.setPower(motorPower - turningPower);
                motorBL.setPower(motorPower + turningPower);
                motorBR.setPower(-motorPower - turningPower);
            } else if (heading == 270) {
                motorFL.setPower(motorPower + turningPower);
                motorFR.setPower(-motorPower - turningPower);
                motorBL.setPower(-motorPower + turningPower);
                motorBR.setPower(motorPower - turningPower);
            }
        }
    }
}