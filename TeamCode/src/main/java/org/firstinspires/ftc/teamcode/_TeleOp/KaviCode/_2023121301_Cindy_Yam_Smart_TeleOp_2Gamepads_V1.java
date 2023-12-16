package org.firstinspires.ftc.teamcode._TeleOp.KaviCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm.ArmInstance;
import org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm.ClawInstance;
import org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm.DroneLauncherInstance;

@TeleOp(name = "2 Gamepads Genshin Impact")
public class _2023121301_Cindy_Yam_Smart_TeleOp_2Gamepads_V1 extends LinearOpMode {

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

        boolean armDown = true;

        waitForStart();

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            x = x * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1a
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = Math.round(((y + x + rx) / denominator));
            double backLeftPower = Math.round(((y - x + rx) / denominator));
            double frontRightPower = Math.round(((y - x - rx) / denominator));
            double backRightPower = Math.round(((y + x - rx) / denominator));

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

            if (gamepad2.y) {
                Claw.Actuate_Claw_Top_Finger("open");
            }
            if (gamepad2.x) {
                Claw.Actuate_Claw_Top_Finger("close");
            }

            if (gamepad2.a) {
                Claw.Actuate_Claw_Bottom_Finger("open");
            }
            if (gamepad2.b) {
                Claw.Actuate_Claw_Bottom_Finger("close");
            }

            if (gamepad2.right_trigger > 0) {
                armDown = false;
                Arm.moveArmBy((int) (Arm_Adjustment_Value * gamepad2.right_trigger));
            } else if (gamepad2.left_trigger > 0) {
                armDown = false;
                Arm.moveArmBy((int) (-Arm_Adjustment_Value * gamepad2.left_trigger));
            }
            if (gamepad2.dpad_up) {
                DroneLauncher.launchDrone();
            }

            if ((Arm.Arm_Motor.getCurrentPosition() > 505) || (Arm.Arm_Motor.getTargetPosition() == 300)){
                Arm.setArmPosTo(500, 0.1);
            }
            if (Arm.Arm_Motor.getCurrentPosition() < 5) {
                Arm.setArmPosTo(5, 0.1);
            }



            //Smart TeleOp

            if (gamepad1.a){
                armSpeed = 0.2;
            }
            else if(gamepad1.b){
                armSpeed = 0.15;
            }

            if (gamepad2.right_bumper) {
                armDown = false;
                Arm.setArmPosTo(300, armSpeed);
                while (Arm.Arm_Motor.isBusy()) {}
                //sleep(750);
                //Claw.Actuate_Claw_Bottom_Finger("open");
                //sleep(500);
                //Claw.Actuate_Claw_Top_Finger("open");


            }

// genshin uid: 642041765
// add me pls !!

            if (gamepad2.left_bumper) {
                if (armDown) {
                    armDown = false;
                    Arm.setArmPosTo(100,0.3);
                } else {
                    armDown = true;
                    Arm.setArmPosTo(5,0.2);
                }
            }

            if (!Arm.Arm_Motor.isBusy() && armDown) {
                sleep(50);
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
            }

            Arm.setArmPosTo(Arm.getCurrentArmPos(), 0.1);
            sleep(50);

            telemetry.addLine("Open Claw Top: y");
            telemetry.addLine("Close Claw Top: x");
            telemetry.addLine("Open Claw Bottom: b");
            telemetry.addLine("Close Claw Bottom: a");
            telemetry.addData("Arm Position: ", Arm.Arm_Motor.getCurrentPosition());
            telemetry.addData("Arm Target Position: ", Arm.getCurrentArmPos());
            telemetry.addData("Arm Target Position: ", Arm.Arm_Motor.getTargetPosition());
            telemetry.addData("Bottom Claw Position: ", Claw.Claw_Bottom_Finger.getPosition());
            telemetry.addData("Drone Launcher Position: ", DroneLauncher.DroneLauncherServo.getPosition());
            telemetry.update();
        }
    }
}
