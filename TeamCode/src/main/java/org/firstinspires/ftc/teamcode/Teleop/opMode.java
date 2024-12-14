package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="opMode", group="Linear OpMode")
public class opMode extends LinearOpMode {
    static final int LEFT_EXTENDER_ENDSTOP = 1695;
    static final int RIGHT_EXTENDER_ENDSTOP = 1695;
    //Jesus recommended it and said it would work
   static final double EXTENDER_SCALING = 1.0/3;
   static final double WRIST_SCALING_DEGREES = 9.0/300;
   static final double GRIPPER_SCALING_DEGREES = 1.0/300;
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftExtender = hardwareMap.get(DcMotor.class, "leftExtender");
        DcMotor rightExtender = hardwareMap.get(DcMotor.class, "rightExtender");
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        Servo gripperServo = hardwareMap.get(Servo.class, "gripperServo");

        leftExtender.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {
            // Forward and Backward controls
            {
                leftFront.setPower(gamepad1.left_stick_y);
                leftBack.setPower(gamepad1.left_stick_y);
                rightBack.setPower(gamepad1.left_stick_y);
                rightFront.setPower(gamepad1.left_stick_y);
            }
            // Left and Right rotation controls
            {
                leftFront.setPower(-gamepad1.right_stick_x);
                leftBack.setPower(-gamepad1.right_stick_x);
                rightBack.setPower(gamepad1.right_stick_x);
                rightFront.setPower(gamepad1.right_stick_x);
            }
           // Strafing left and Strafing right
            {
                if (gamepad1.dpad_left) {
                    leftFront.setPower(1);
                    leftBack.setPower(-1);
                    rightBack.setPower(1);
                    rightFront.setPower(-1);
                }
                if (gamepad1.dpad_right) {
                    leftFront.setPower(-1);
                    leftBack.setPower(1);
                    rightBack.setPower(-1);
                    rightFront.setPower(1);
                }
            }
            // Extender controls

            {
                double powerDifferential =
                        (
                                leftExtender.getCurrentPosition()/ LEFT_EXTENDER_ENDSTOP - rightExtender.getCurrentPosition()/ RIGHT_EXTENDER_ENDSTOP
                        ) / 2 * 10;

                //1 if going out, -1 if going in
                int direction = gamepad2.left_stick_y > 0 ? 1 : -1;

                leftExtender.setPower(Clamp((-gamepad2.left_stick_y - direction * powerDifferential)));
                rightExtender.setPower(Clamp((-gamepad2.left_stick_y + direction * powerDifferential)));
            }
            // To set the wrist servo A is down, B is forward
            {
                if (gamepad2.a) {
                    wristServo.setPosition(.8);
                }
                if (gamepad2.b) {
                    wristServo.setPosition(0.3);
                }
            }
            // Gripper controls Y is open, X is close
            {
                if (gamepad2.y){
                    gripperServo.setPosition(120 * GRIPPER_SCALING_DEGREES);
                }
                if (gamepad2.x){
                    gripperServo.setPosition(20 * GRIPPER_SCALING_DEGREES);
                }
            }
            // To reset all encoders on the bot
            if (gamepad2.start){
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addData("leftFront Pos:", leftFront.getCurrentPosition());
            telemetry.addData("leftBack Pos:", leftBack.getCurrentPosition());
            telemetry.addData("rightBack Pos:", rightBack.getCurrentPosition());
            telemetry.addData("rightFront  Pos:", rightFront.getCurrentPosition());
            telemetry.addData("leftExtender Pos", leftExtender.getCurrentPosition());
            telemetry.addData("rightExtender Pos", rightExtender.getCurrentPosition());
            telemetry.addData("Wrist Pos: ", wristServo.getPosition());
            telemetry.addData("Left stick Y: ", gamepad2.left_stick_y);
            telemetry.addData("Right stick Y: ", gamepad2.right_stick_y);
            telemetry.update();
        }
    }
    static double Clamp(double value){
        return Math.min(1,Math.max(-1, value));
    }
}



