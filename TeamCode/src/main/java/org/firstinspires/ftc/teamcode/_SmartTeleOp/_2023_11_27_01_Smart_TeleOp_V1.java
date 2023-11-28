package org.firstinspires.ftc.teamcode._SmartTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "V1 Smart TeleOp")
public class _2023_11_27_01_Smart_TeleOp_V1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor rightOdometryPod = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor leftOdometryPod = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor backOdometryPod = hardwareMap.get(DcMotor.class, "motorBR");

        int rightOdometryPodTicks = 0;
        int leftOdometryPodTicks = 0;
        int backOdometryPodTicks = 0;

        double rightOdometryPodInches = 0;
        double leftOdometryPodInches =  0;
        double backOdometryPodInches = 0;

        double robotHeading = 0;

        double robotX = 0;
        double robotY = 0;

        double r_t = 0;
        double r_s = 0;

        waitForStart();

        while (opModeIsActive()) {
            rightOdometryPodTicks = rightOdometryPod.getCurrentPosition();
            leftOdometryPodTicks = leftOdometryPod.getCurrentPosition();
            backOdometryPodTicks = backOdometryPod.getCurrentPosition();

            rightOdometryPodInches = calculateOdometryInches(rightOdometryPodTicks);
            leftOdometryPodInches= calculateOdometryInches(leftOdometryPodTicks);
            backOdometryPodInches = calculateOdometryInches(backOdometryPodTicks);

            robotHeading = (rightOdometryPodInches-leftOdometryPodInches/12);

            if (robotHeading == 0) {
                robotX = backOdometryPodInches;
                robotY = (leftOdometryPodInches + rightOdometryPodInches)/2;
            } else {
                r_t = (6 * (leftOdometryPodInches + rightOdometryPodInches))/(rightOdometryPodInches - leftOdometryPodInches);
                r_s = (backOdometryPodInches/robotHeading) - 9;
                robotX = r_t * (Math.cos(robotHeading) - 1) + r_s * Math.sin(robotHeading);
                robotY = r_t * Math.sin(robotHeading) + (r_s * (1 - Math.cos(robotHeading)));
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            x = x * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = Math.round(((y + x + rx) / denominator));
            double backLeftPower = Math.round(((y - x + rx) / denominator));
            double frontRightPower = Math.round(((y - x - rx) / denominator));
            double backRightPower = Math.round(((y + x - rx) / denominator));

            frontLeftMotor.setPower(frontLeftPower * 0.8);
            backLeftMotor.setPower(backLeftPower * 0.8);
            frontRightMotor.setPower(frontRightPower * 0.8);
            backRightMotor.setPower(backRightPower * 0.8);

            // ADDED CODE - sends info about current servo position to driver station
            telemetry.addData("y (left stick y): ", y);
            telemetry.addData("x (left stick x): ", x);
            telemetry.addData("rx (right stick x): ", rx);

            telemetry.addData("Front Left Power: ", frontLeftPower);
            telemetry.addData("Front Right Power: ", frontRightPower);
            telemetry.addData("Back Left Power: ", backLeftPower);
            telemetry.addData("Back Right Power: ", backRightPower);

            telemetry.addData("Right Odometry Pod Measurement: ", "%d ticks", rightOdometryPodTicks);
            telemetry.addData("Left Odometry Pod Measurement: ", "%d ticks", leftOdometryPodTicks);
            telemetry.addData("Back Odometry Pod Measurement: ", "%d ticks", backOdometryPodTicks);

            telemetry.addData("Right Odometry Pod Inch: ", "%f inches", rightOdometryPodInches);
            telemetry.addData("Left Odometry Pod Inch: ", "%f inches", leftOdometryPodInches);
            telemetry.addData("Back Odometry Pod Inch: ", "%f inches", backOdometryPodInches);

            telemetry.addData("Robot Heading: ", robotHeading);

            telemetry.addData("x = ", robotX);
            telemetry.addData("y = ", robotY);

            telemetry.update();
        }
    }

    public double calculateOdometryInches(int odometryTickMeasurement) {
        double odometryTickMeasurementDecimal = Double.valueOf(odometryTickMeasurement);
        double odometryInchMeasurement = (odometryTickMeasurementDecimal/2000) * (2 * Math.PI * 0.944882);
        return odometryInchMeasurement;
    }
}
