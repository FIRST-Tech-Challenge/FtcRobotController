package org.firstinspires.ftc.teamcode._TeleOp.Virtual_4Bar;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "_ New Arm Claw V1 (No Drivebase)")
public class _2023_11_27_01_New_Arm_Claw_NoDB_V1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        Servo clawTop = hardwareMap.servo.get("clawTop");
        Servo clawBottom = hardwareMap.servo.get("clawBottom");

        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        clawTop.setDirection(Servo.Direction.FORWARD);
        clawBottom.setDirection(Servo.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawTop.scaleRange(0, 1);
        clawBottom.scaleRange(0, 1);

        //clawBottom.setPosition(0);
        //clawTop.setPosition(0);

        int liftTargetPosition = 0;
        //double openClaw = 0.4; //increase to open more
        //double closeClaw = 0.2; //decrease to close more
        //boolean armDown = true;

        waitForStart();

        //clawTop.setPosition(openClaw);
        //clawBottom.setPosition(openClaw);

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                liftTargetPosition += 1;
            } else if (gamepad1.left_trigger > 0) {
                liftTargetPosition -= 1;
            }

            /*
            if (gamepad1.right_bumper) {
                for (int liftPos = 0; liftPos <= 500; liftPos = liftPos + 4) {
                    liftTargetPosition = liftPos;
                    arm.setTargetPosition(liftTargetPosition);
                    arm.setPower(0.7);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(12);
                }
                clawBottom.setPosition(openClaw);
                sleep(700);
                clawTop.setPosition(openClaw);

            }
            if (gamepad1.left_bumper) {
                if (armDown) {
                    liftTargetPosition = 300;
                    armDown = false;
                    arm.setTargetPosition(liftTargetPosition);
                    arm.setPower(0.07);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    clawTop.setPosition(openClaw);
                    clawBottom.setPosition(openClaw);
                } else {
                    liftTargetPosition = 5;
                    armDown = true;
                    arm.setTargetPosition(liftTargetPosition);
                    arm.setPower(0.07);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    clawTop.setPosition(closeClaw);
                    clawBottom.setPosition(closeClaw);
                }
            }

            if (!arm.isBusy()) {
                arm.setTargetPosition(liftTargetPosition);
                arm.setPower(0.7);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
             */


            if (gamepad1.y) {
                clawTop.setPosition(0.5);
            }
            if (gamepad1.x) {
                clawTop.setPosition(0.3);
            }

            if (gamepad1.b) {
                clawBottom.setPosition(0.4);
            }
            if (gamepad1.a) {
                clawBottom.setPosition(0.2);
            }

            arm.setTargetPosition(liftTargetPosition);
            arm.setPower(0.7);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Claw Top Position: ", clawTop.getPosition());
            telemetry.addData("Claw Bottom Position: ", clawBottom.getPosition());

            telemetry.addData("Arm position: ", arm.getCurrentPosition());
            telemetry.addData("Arm Target Position Requested: ", liftTargetPosition);
            telemetry.addData("Arm Actual Target Position: ", arm.getTargetPosition());

            telemetry.update();
        }
    }
}