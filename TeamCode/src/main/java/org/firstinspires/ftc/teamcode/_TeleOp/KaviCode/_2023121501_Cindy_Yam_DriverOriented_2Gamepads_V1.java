package org.firstinspires.ftc.teamcode._TeleOp.KaviCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm.ArmInstance;
import org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm.ClawInstance;
import org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm.DroneLauncherInstance;

@TeleOp(name = "2 Gamepads Driver Oriented")
public class _2023121501_Cindy_Yam_DriverOriented_2Gamepads_V1 extends LinearOpMode {

    private int Arm_Adjustment_Value = 50;

    private double Driving_Speed = 0.85;
    double armSpeed = 0.15;
    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        ArmInstance Arm = new ArmInstance();
        ClawInstance Claw = new ClawInstance();
        DroneLauncherInstance DroneLauncher = new DroneLauncherInstance();

        Arm.initializeArm(hardwareMap);
        Claw.initializeClaw(hardwareMap);
        DroneLauncher.initializeDroneLauncher(hardwareMap);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        boolean armDown = true;
        boolean backboardPos = false;

        waitForStart();

        while (opModeIsActive()) {
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

            if (gamepad1.dpad_up) {
                Driving_Speed = 0.85;
            }
            else if (gamepad1.dpad_down) {
                Driving_Speed = 0.5;
            }

            frontLeftMotor.setPower(frontLeftPower * Driving_Speed);
            backLeftMotor.setPower(backLeftPower * Driving_Speed);
            frontRightMotor.setPower(frontRightPower * Driving_Speed);
            backRightMotor.setPower(backRightPower * Driving_Speed);

            if (gamepad1.y) {
                Claw.Actuate_Claw_Top_Finger("toggle");
            }
            if (gamepad1.a) {
                Claw.Actuate_Claw_Top_Finger("toggle");
            }

            if (gamepad1.right_trigger > 0) {
                armDown = false;
                Arm.moveArmBy((int) (Arm_Adjustment_Value * gamepad1.right_trigger));
            } else if (gamepad1.left_trigger > 0) {
                armDown = false;
                Arm.moveArmBy((int) (-Arm_Adjustment_Value * gamepad1.left_trigger));
            }
            if (gamepad1.b) {
                DroneLauncher.launchDrone();
            }

            if ((Arm.Arm_Motor.getCurrentPosition() > 505) || (Arm.Arm_Motor.getTargetPosition() == 300)){
                Arm.setArmPosTo(500, 0.1);
            }
            if (Arm.Arm_Motor.getCurrentPosition() < 5) {
                Arm.setArmPosTo(5, 0.1);
            }

            //Smart TeleOp

            /*if (gamepad1.a){
                armSpeed = 0.2;
            }
            else if(gamepad1.b){
                armSpeed = 0.15;
            }
             */

            if (gamepad1.right_bumper) {
                armDown = false;
                if (backboardPos == false) {
                    backboardPos = true;
                    Arm.setArmPosTo(300, armSpeed);
                }else {
                    backboardPos = false;
                    Claw.Actuate_Claw_Bottom_Finger("open");

                    backLeftMotor.setPower(Driving_Speed);
                    backRightMotor.setPower(-Driving_Speed);
                    frontLeftMotor.setPower(-Driving_Speed);
                    frontRightMotor.setPower(Driving_Speed);
                    sleep(700);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);

                    Claw.Actuate_Claw_Top_Finger("open");
                }



            }

// genshin uid: 642041765
// add me pls !!

            if (gamepad1.left_bumper) {
                if (!armDown) {
                    Driving_Speed = 0.15;
                    backLeftMotor.setPower(-Driving_Speed);
                    backRightMotor.setPower(-Driving_Speed);
                    frontLeftMotor.setPower(-Driving_Speed);
                    frontRightMotor.setPower(-Driving_Speed);

                    sleep(700);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);

                    armDown = true;
                    Arm.setArmPosTo(100, 0.3);

                    Driving_Speed = 0.85;
                }
            }

            if (!Arm.Arm_Motor.isBusy() && armDown) {
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                Driving_Speed = 0.85;
                armDown = false;
            }

            Arm.setArmPosTo(Arm.getCurrentArmPos(), 0.1);
            sleep(50);

            telemetry.addLine("Open Claw Top: y");
            telemetry.addLine("Close Claw Top: x");
            telemetry.addLine("Open Claw Bottom: a");
            telemetry.addLine("Close Claw Bottom: b");
            telemetry.addData("Arm Position: ", Arm.Arm_Motor.getCurrentPosition());
            telemetry.addData("Arm Target Position: ", Arm.getCurrentArmPos());
            telemetry.addData("Arm Target Position: ", Arm.Arm_Motor.getTargetPosition());
            telemetry.addData("Bottom Claw Position: ", Claw.Claw_Bottom_Finger.getPosition());
            telemetry.addData("Drone Launcher Position: ", DroneLauncher.DroneLauncherServo.getPosition());
            telemetry.update();
        }
    }
}
