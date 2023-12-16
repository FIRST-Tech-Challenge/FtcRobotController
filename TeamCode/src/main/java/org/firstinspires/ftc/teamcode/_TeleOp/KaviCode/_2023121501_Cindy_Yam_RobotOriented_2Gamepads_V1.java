package org.firstinspires.ftc.teamcode._TeleOp.KaviCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm.ArmInstance;
import org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm.ClawInstance;
import org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm.DroneLauncherInstance;

@TeleOp(name = "2 Gamepads Robot Oriented")
public class _2023121501_Cindy_Yam_RobotOriented_2Gamepads_V1 extends LinearOpMode {

    private int Arm_Adjustment_Value = 50;

    private double Driving_Speed = 0.85;
    double armSpeed = 0.15;
    @Override
    public void runOpMode() throws InterruptedException {
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

        boolean armInAction = true;
        boolean backboardPos = false;

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = Math.round(((y + x + rx) / denominator));
            double backLeftPower = Math.round(((y - x + rx) / denominator));
            double frontRightPower = Math.round(((y - x - rx) / denominator));
            double backRightPower = Math.round(((y + x - rx) / denominator));

            /*if (gamepad1.dpad_up) {
                Driving_Speed = 0.85;
            }
            else if (gamepad1.dpad_down) {
                Driving_Speed = 0.5;
            }

             */

            frontLeftMotor.setPower(frontLeftPower * Driving_Speed);
            backLeftMotor.setPower(backLeftPower * Driving_Speed);
            frontRightMotor.setPower(frontRightPower * Driving_Speed);
            backRightMotor.setPower(backRightPower * Driving_Speed);

            if (gamepad1.y) {
                Claw.Actuate_Claw_Top_Finger("toggle");
            }
            if (gamepad1.a) {
                Claw.Actuate_Claw_Bottom_Finger("toggle");
            }

            if (gamepad1.right_trigger > 0) {
                armInAction = false;
                Arm.moveArmBy((int) (Arm_Adjustment_Value * gamepad1.right_trigger));
            } else if (gamepad1.left_trigger > 0) {
                armInAction = false;
                Arm.moveArmBy((int) (-Arm_Adjustment_Value * gamepad1.left_trigger));
            }
            if (gamepad1.dpad_down) {
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
                armInAction = true;
                if (!backboardPos) {
                    backboardPos = true;
                    Arm.setArmPosTo(300, armSpeed);
                    while (Arm.Arm_Motor.isBusy()) {}
                }else {
                    backboardPos = false;
                    Claw.Actuate_Claw_Bottom_Finger("open");
                    sleep(750);

                    Driving_Speed = 0.2;
                    backLeftMotor.setPower(Driving_Speed);
                    backRightMotor.setPower(-Driving_Speed);
                    frontLeftMotor.setPower(-Driving_Speed);
                    frontRightMotor.setPower(Driving_Speed);
                    sleep(800);

                    Driving_Speed = 0.85;
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    while (Arm.Arm_Motor.isBusy()) {}

                    Claw.Actuate_Claw_Top_Finger("open");
                }



            }

// genshin uid: 642041765
// add me pls !!
            if (gamepad1.left_bumper) {
                if (armInAction) {
                    Arm.setArmPosTo(100,0.15);
                    armInAction = false;
                    while (Arm.Arm_Motor.isBusy()) {}
                } else {
                    armInAction = true;
                    Claw.Actuate_Claw_Bottom_Finger("open");
                    Claw.Actuate_Claw_Top_Finger("open");
                    sleep(700);
                    Arm.setArmPosTo(5, 0.15);
                    while (Arm.Arm_Motor.isBusy()) {}
                }
            }

            if (gamepad1.b) {
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

                Driving_Speed = 0.85;

                while (Arm.Arm_Motor.isBusy()) {}
            }

            if (!Arm.Arm_Motor.isBusy() && armInAction) {
                armInAction = false;
                sleep(50);
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                sleep(150);
                Arm.setArmPosTo(100,0.15);
                while (Arm.Arm_Motor.isBusy()) {}
            }

            Arm.setArmPosTo(Arm.getCurrentArmPos(), 0.1);

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
