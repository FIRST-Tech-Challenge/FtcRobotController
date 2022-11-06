package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public abstract class BaseAutonomous extends BaseOpMode {

    // this method will allow the robot to drive straight in a specified direction using the IMU given a specified heading and distance
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

            telemetry.addData("fl", String.valueOf(motorFL.getPower()));
            telemetry.addData("fr", String.valueOf(motorFR.getPower()));
            telemetry.addData("bl", String.valueOf(motorBL.getPower()));
            telemetry.addData("br", String.valueOf(motorBR.getPower()));
            telemetry.addData("traveled", String.valueOf(distanceTraveled));
            telemetry.addData("x", String.valueOf(imu.getPosition().x));
            telemetry.addData("y", String.valueOf(imu.getPosition().y));
            telemetry.update();
        }
    }
}


//    // this method will allow the robot to drive straight in a specified direction using the IMU given a specified heading and distance
//    public void driveOmniInches(int heading, double targetDistance) {
//        boolean distanceReached = false;
//
//        double xPosition = 0, yPosition = 0;
//        double distanceTraveled, angleDeviation, motorPower, turningPower;
//
//        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//        double headingRadians = Math.toRadians(heading);
//
//        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        while (!distanceReached && opModeIsActive()) {
//            angleDeviation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle;
//
//            distanceTraveled = Math.sqrt(Math.pow(xPosition, 2) + Math.pow(yPosition, 2));
//
//            turningPower = angleDeviation / 50;
//
//            // todo - max linear acceleration where encoders don't slip, then 0, then max linear deceleration where encoders don't slip
//            motorPower = Math.max(Math.sqrt(Math.sin(distanceTraveled / targetDistance * Math.PI)) * 0.5, 0.1);
//
//            // todo - make work for any heading
//            if (headingRadians == 0) {
//                motorFL.setPower(motorPower + turningPower);
//                motorFR.setPower(motorPower - turningPower);
//                motorBL.setPower(motorPower + turningPower);
//                motorBR.setPower(motorPower - turningPower);
//            } else if (headingRadians == Math.PI / 2) {
//                motorFL.setPower(-motorPower + turningPower);
//                motorFR.setPower(-motorPower - turningPower);
//                motorBL.setPower(-motorPower + turningPower);
//                motorBR.setPower(-motorPower - turningPower);
//            } else if (headingRadians == Math.PI) {
//                motorFL.setPower(-motorPower + turningPower);
//                motorFR.setPower(motorPower - turningPower);
//                motorBL.setPower(motorPower + turningPower);
//                motorBR.setPower(-motorPower - turningPower);
//            } else if (headingRadians == 3 * Math.PI / 2) {
//                motorFL.setPower(motorPower + turningPower);
//                motorFR.setPower(-motorPower - turningPower);
//                motorBL.setPower(-motorPower + turningPower);
//                motorBR.setPower(motorPower - turningPower);
//            }
//
//            xPosition = (Constants.CHASSIS_INCHES_PER_TICK * (motorFL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBL.getCurrentPosition() + motorBR.getCurrentPosition()) / 4.0);
//            yPosition = (Constants.CHASSIS_INCHES_PER_TICK * (motorFL.getCurrentPosition() - motorFR.getCurrentPosition() - motorBL.getCurrentPosition() + motorBR.getCurrentPosition()) / 4.0);
//
//            if (distanceTraveled > targetDistance) {
//                motorFL.setPower(0.0);
//                motorFR.setPower(0.0);
//                motorBL.setPower(0.0);
//                motorBR.setPower(0.0);
//
//                distanceReached = true;
//            }
//
//            telemetry.addLine(String.valueOf(xPosition));
//            telemetry.addLine(String.valueOf(yPosition));
//            telemetry.addLine(String.valueOf(distanceTraveled));
//            telemetry.addLine(String.valueOf(distanceReached));
//            telemetry.addLine(String.valueOf(angleDeviation));
//            telemetry.addLine(String.valueOf(turningPower));
//            telemetry.addLine(String.valueOf(motorPower));
//            telemetry.update();
//        }
//    }