package org.firstinspires.ftc.teamcode.versionCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele Operated Meet 0 Final")
public class _20231013_Kavi_Gupta_FTC_48_Tele_Operated_Final_Code_Meet_00 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

        DcMotor lift = hardwareMap.dcMotor.get("lift");
        Servo droneServo = hardwareMap.servo.get("drone");
        Servo wristServo = hardwareMap.servo.get("wrist");
        Servo clawServo = hardwareMap.servo.get("claw");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        droneServo.setDirection(Servo.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB backward
        imu.initialize(parameters);

        droneServo.scaleRange(0, 1);
        droneServo.setPosition(0.85);
        wristServo.scaleRange(0, 1);
        wristServo.setPosition(0);
        int liftTargetPosition = 5;
        double wristTargetPos = 0;
        float claw = 0;

        waitForStart();

        while (opModeIsActive()) {
            //gamepad 1 - drive base
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.x) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX + rx) / denominator);
            double backLeftPower = ((rotY - rotX + rx) / denominator);
            double frontRightPower = ((rotY - rotX - rx) / denominator);
            double backRightPower = ((rotY + rotX - rx) / denominator);

            frontLeftMotor.setPower(frontLeftPower/2);
            backLeftMotor.setPower(backLeftPower/2);
            frontRightMotor.setPower(frontRightPower/2);
            backRightMotor.setPower(backRightPower/2);

            //gamepad 2 - mechanisms
            //lift code
            double liftPower = lift.getPower();

            if (gamepad2.right_trigger > 0) {
                liftTargetPosition += 5;
            }else if (gamepad2.left_trigger > 0) {
                liftTargetPosition -= 5;
            }
            if (liftTargetPosition > 962) {
                liftTargetPosition = 957;
            }
            if (liftTargetPosition < 0) {
                liftTargetPosition = 5;
            }

            lift.setTargetPosition(liftTargetPosition);
            lift.setPower(1);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //wrist code

            if (wristTargetPos > 1){
                wristTargetPos = 1;
            } else if (wristTargetPos < 0){
                wristTargetPos = 0;
            }

            if (lift.getCurrentPosition() >= 0 && lift.getCurrentPosition() < 500) {
                wristTargetPos = 0;
                wristServo.setPosition(wristTargetPos);

            } else if (lift.getCurrentPosition() >= 500) {
                wristTargetPos = 1;
                wristServo.setPosition(wristTargetPos);
            }

            //claw code

            float clawOpenPosition = 1;
            float clawClosedPosition = 0;
            double clawActualPosition = clawServo.getPosition();
            String clawStatus = null;

            if (clawActualPosition >= clawOpenPosition - 0.25 && clawActualPosition <= clawClosedPosition/2) {
                clawStatus = "open";
            } else if (clawActualPosition <= clawClosedPosition + 0.25 && clawActualPosition > clawClosedPosition/2) {
                clawStatus = "closed";
            }

            if (gamepad2.b) {
                if (clawStatus == "open") {
                    claw = clawClosedPosition;
                } else if (clawStatus == "closed") {
                    claw = clawOpenPosition;
                }
            }

            if (gamepad2.dpad_up) {
                claw += 0.05;
                clawServo.setPosition(claw);
            }else if (gamepad2.dpad_down) {
                claw -= 0.05;
                clawServo.setPosition(claw);
            }

            //drone launcher code

            double droneServoPosition = droneServo.getPosition();

            if (gamepad2.a) {
                droneServo.setPosition(1);
                sleep(1500);
                droneServo.setPosition(0.85);
            }

            // ADDED CODE - sends info about current servo position to driver station
            telemetry.addData("Servo Position: ", droneServoPosition);
            telemetry.addData("Wrist Position: ", wristServo.getPosition());
            telemetry.addData("Lift position: ", lift.getCurrentPosition());
            telemetry.addData("Target Position Requested: ", liftTargetPosition);
            telemetry.addData("Actual Target Position: ", lift.getTargetPosition());
            telemetry.addData("Lift power: ", liftPower);
            telemetry.addData("Claw Target: ", claw);
            telemetry.addData("Claw Actual: ", clawActualPosition);
            telemetry.update();
        }
    }
}