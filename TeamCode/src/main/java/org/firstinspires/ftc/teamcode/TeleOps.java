package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.Math;
//import java.lang.Float;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "TeleOps 2020-2021", group = "")
public class TeleOps extends LinearOpMode {
    public DcMotor rightShooter;
    public TouchSensor liftButtonBot;
    public CRServo liftservo;
    public TouchSensor liftButtonTop;
    public DcMotor picker;
    public Servo pusher;
    public DcMotor leftShooter;
    public DcMotor Right_Front_Wheel;
    public DcMotor Left_Front_Wheel;
    public DcMotor Right_Rear_Wheel;
    public DcMotor Left_Rear_Wheel;

    private Robot robot;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        liftButtonBot = hardwareMap.get(TouchSensor.class, "liftButtonBot");
        liftservo = hardwareMap.get(CRServo.class, "liftservo");
        liftButtonTop = hardwareMap.get(TouchSensor.class, "liftButtonTop");
        picker = hardwareMap.get(DcMotor.class, "picker");
        pusher = hardwareMap.get(Servo.class, "pusher");
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");

        Left_Rear_Wheel = hardwareMap.get(DcMotor.class, "Left_Rear_Wheel");
        Left_Front_Wheel = hardwareMap.get(DcMotor.class, "Left_Front_Wheel");
        Right_Rear_Wheel = hardwareMap.get(DcMotor.class, "Right_Rear_Wheel");
        Right_Front_Wheel = hardwareMap.get(DcMotor.class, "Right_Front_Wheel");

        robot = new Robot();
        robot.Init(hardwareMap, telemetry, false);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //This one takes care of Forward , Reverse, Right and Left
                if (gamepad2.a && !liftButtonBot.isPressed()) {
                    while (!liftButtonBot.isPressed()) {
                        liftservo.setPower(-1);
                    }
                } else if (gamepad2.y && !liftButtonTop.isPressed()) {
                    while (!liftButtonTop.isPressed()) {
                        liftservo.setPower(1);
                        picker.setPower(0);
                    }
                } else {
                    liftservo.setPower(0);
                }
                if (gamepad2.x && liftButtonBot.isPressed()) {
                    picker.setPower(1);
                }
                if (gamepad2.b) {
                    picker.setPower(0);
                }
                if (gamepad2.left_bumper && liftButtonTop.isPressed()) {
                    pusher.setPosition(1);
                    sleep(500);
                    pusher.setPosition(-1);
                }
                if (liftButtonTop.isPressed()) {
                    leftShooter.setPower(1);
                    rightShooter.setPower(1);
                } else {
                    leftShooter.setPower(0);
                    rightShooter.setPower(0);
                }
            }
                if (Math.abs(gamepad1.left_stick_y) > Math.abs((gamepad1.left_stick_x))) {
                    if (gamepad1.left_stick_y < 0) {
                        robot.Forward(Math.abs(gamepad1.left_stick_y));
                    } else if (gamepad1.left_stick_y > 0) {
                        robot.Reverse(Math.abs(gamepad1.left_stick_y));
                    }
                } else if (Math.abs(gamepad1.left_stick_y) < Math.abs((gamepad1.left_stick_x))) {
                    if (gamepad1.left_stick_x > 0) {

                        robot.Right(Math.abs(gamepad1.left_stick_x));
                    } else if (gamepad1.left_stick_x < 0) {
                        robot.Left(Math.abs(gamepad1.left_stick_x));
                    }
                } else {
                    if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                        robot.Stop();
                    }
                }

                //This one takes care of Diagonal Right Up , Diagonal Right Down, Diagonal Left Up, Diagonal Left Down
                if (gamepad1.right_stick_x < 0 && gamepad1.right_stick_y < 0) {
                    if (Math.abs(gamepad1.right_stick_x) >= Math.abs(gamepad1.right_stick_y)) {
                        robot.Diagonal_Left_Up(Math.abs(gamepad1.right_stick_x));
                    } else {
                        robot.Diagonal_Left_Up(Math.abs(gamepad1.right_stick_y));
                    }
                } else if (gamepad1.right_stick_x < 0 && gamepad1.right_stick_y > 0) {
                    if (Math.abs(gamepad1.right_stick_x) >= Math.abs(gamepad1.right_stick_y)) {
                        robot.Diagonal_Left_Down(Math.abs(gamepad1.right_stick_x));
                    } else {
                        robot.Diagonal_Left_Down(Math.abs(gamepad1.right_stick_y));
                    }
                } else if (gamepad1.right_stick_x > 0 && gamepad1.right_stick_y < 0) {
                    if (Math.abs(gamepad1.right_stick_x) >= Math.abs(gamepad1.right_stick_y)) {
                        robot.Diagonal_Right_Up(Math.abs(gamepad1.right_stick_x));
                    } else {
                        robot.Diagonal_Right_Up(Math.abs(gamepad1.right_stick_y));
                    }
                } else if (gamepad1.right_stick_x > 0 && gamepad1.right_stick_y > 0) {
                    if (Math.abs(gamepad1.right_stick_x) >= Math.abs(gamepad1.right_stick_y)) {
                        robot.Diagonal_Right_Down(Math.abs(gamepad1.right_stick_x));
                    } else {
                        robot.Diagonal_Right_Down(Math.abs(gamepad1.right_stick_y));
                    }
                } else {
                    if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                        robot.Stop();
                    }
                }

                //Slide Code is Here
                if (gamepad1.right_trigger <= 1 && gamepad1.right_trigger > 0) {
                    robot.Slide_Right(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger <= 1 && gamepad1.left_trigger > 0) {
                    robot.Slide_Left(gamepad1.left_trigger);
                } else {
                    robot.Stop();
                }


            }

            telemetry.update();
        }
    }


