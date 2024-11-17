package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Autonomous
public class Auto extends LinearOpMode {
    private Servo Hand_Rotator_Servo; // rotates the hand
    private Servo Hand_Servo;// open and closes the hand
    private DcMotor Front_Left_Wheel;
    private DcMotor Front_Right_Wheel;
    private DcMotor Back_Left_Wheel;
    private DcMotor Back_Right_Wheel;
    private DcMotor Arm_Motor;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        Updates Data
         */
        //create failsafe so if joysticks are not being used then switch between joystick and dpad for checking movement
        initialize_motors();//motor setup
        initialize_servos();//servo setup
        initialize_direction(); //direction setup
        Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wait for start to be pressed
        waitForStart();
        //
        //if op mode is active then we can continue
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                display(); //telemetry data
                Arm_Motor.setTargetPosition(180);
                Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }

    private void initialize_motors() {
        Front_Left_Wheel = hardwareMap.get(DcMotor.class, "front_left_wheel");
        Back_Left_Wheel = hardwareMap.get(DcMotor.class, "back_left_wheel");
        Front_Right_Wheel = hardwareMap.get(DcMotor.class, "front_right_wheel");
        Back_Right_Wheel = hardwareMap.get(DcMotor.class, "back_right_wheel");
        Arm_Motor = hardwareMap.get(DcMotor.class, "arm_control");
    }

    private void initialize_servos() {
        Hand_Rotator_Servo = hardwareMap.get(Servo.class, "hand_rotator");
        Hand_Servo = hardwareMap.get(Servo.class, "hand_servo");
    }

    private void initialize_direction() {
        Back_Left_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        Back_Right_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        Front_Left_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Front_Right_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Hand_Rotator_Servo.setDirection(Servo.Direction.REVERSE);
        Hand_Servo.setDirection(Servo.Direction.FORWARD);

    }
    private void display() {
        telemetry.addData("arm position", Arm_Motor.getCurrentPosition());
        telemetry.update();
    }
}
