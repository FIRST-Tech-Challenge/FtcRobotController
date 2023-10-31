package org.firstinspires.ftc.teamcode.Developing_Code;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "V2 Arm Claw")
public class _2023_10_27_02_Arm_Claw_V2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

        DcMotor arm = hardwareMap.dcMotor.get("arm");
        Servo droneServo = hardwareMap.servo.get("drone");
        Servo clawLeft = hardwareMap.servo.get("clawLeft");
        Servo clawRight = hardwareMap.servo.get("clawRight");
        Servo axle = hardwareMap.servo.get("axle");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        droneServo.setDirection(Servo.Direction.FORWARD);
        clawLeft.setDirection(Servo.Direction.FORWARD);
        clawRight.setDirection(Servo.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB backward
        imu.initialize(parameters);
        droneServo.scaleRange(0, 1);
        droneServo.setPosition(0.85);

        clawLeft.scaleRange(0, 1);
        clawRight.scaleRange(0, 1);
        
        axle.scaleRange(0, 1);
        axle.setPosition(0);

        //clawRight.setPosition(0);
        //clawLeft.setPosition(0);

        int liftTargetPosition = 0;
        double openClaw = 0.4; //increase to open more
        double closeClaw = 0.2; //decrease to close more

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

            frontLeftMotor.setPower(frontLeftPower * 0.65);
            backLeftMotor.setPower(backLeftPower * 0.65);
            frontRightMotor.setPower(frontRightPower * 0.65);
            backRightMotor.setPower(backRightPower * 0.65);

            double liftPower = arm.getPower();

            if (gamepad1.right_trigger > 0) {
                liftTargetPosition += 5;
            } else if (gamepad1.left_trigger > 0) {
                liftTargetPosition -= 5;
            }
            if (liftTargetPosition > 967) {
                liftTargetPosition = 962;
            }
            if (liftTargetPosition < 0) {
                liftTargetPosition = 0;
            }

            if (gamepad1.right_bumper) {
                liftTargetPosition = 962;
                arm.setTargetPosition(liftTargetPosition);
                arm.setPower(1);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (arm.isBusy()) {}
                arm.setPower(0);

                clawLeft.setPosition(openClaw);
                clawRight.setPosition(openClaw);
                
                sleep(800);
                axle.setPosition(1);
            }
            if (gamepad1.left_bumper) {
                liftTargetPosition = 0;
                arm.setTargetPosition(liftTargetPosition);
                arm.setPower(1);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                axle.setPosition(0);
            }
            
            arm.setTargetPosition(liftTargetPosition);
            arm.setPower(1);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double clawLeftTarget = 0;

            if (gamepad1.b) {
                clawLeft.setPosition(closeClaw);
                clawRight.setPosition(closeClaw);
            }
            if (gamepad1.a) {
                clawLeft.setPosition(openClaw);
                clawRight.setPosition(openClaw);
            }

            if (gamepad1.dpad_up) {
                axle.setPosition(1);
            }
            if (gamepad1.dpad_down) {
                axle.setPosition(0);
            }

            telemetry.addData("Claw Left Target: ", clawLeftTarget);
            telemetry.addData("Claw Right Target: ", closeClaw);

            double droneServoPosition = droneServo.getPosition();

            if (gamepad1.y) {
                droneServo.setPosition(1);
                sleep(1500);
                droneServo.setPosition(0.85);
            }

            // ADDED CODE - sends info about current servo position to driver station
            telemetry.addData("Drone Servo Position: ", droneServoPosition);

            telemetry.addData("Claw Left Position: ", clawLeft.getPosition());
            telemetry.addData("Claw Right Position: ", clawRight.getPosition());

            telemetry.addData("Axle Position:", axle.getPosition());

            telemetry.addData("Arm position: ", arm.getCurrentPosition());
            telemetry.addData("Arm power: ", liftPower);
            telemetry.addData("Arm Target Position Requested: ", liftTargetPosition);
            telemetry.addData("Arm Actual Target Position: ", arm.getTargetPosition());

            telemetry.update();
        }
    }
}