package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@TeleOp
public class Main extends LinearOpMode {


    //private static volatile HashMap<String, Float> ControllerData = new HashMap<String, Float>();
    private Servo Hand_Rotator_Servo; // rotates the hand
    private Servo Hand_Servo;// open and closes the hand
    private DcMotor Front_Left_Wheel;
    private DcMotor Front_Right_Wheel;
    private DcMotor Back_Left_Wheel;
    private  DcMotor Back_Right_Wheel;
    private DcMotor Arm_Motor;
    private double dist_to_move = 0;
    //private Controls_Update ControlsUpdate = new Controls_Update();
    //private Data_Update DataUpdate = new Data_Update();
    //private Thread dta = new Thread(DataUpdate);
    @Override
    public void runOpMode() throws InterruptedException {
        /*
        Updates Data
         */
        //create failsafe so if joysticks are not being used then switch between joystick and dpad for checking movement
        initialize_motors();//motor setup
        initialize_servos();//servo setup
        //wait for start to be pressed
        waitForStart();
        //if op mode is active then we can continue
        if(opModeIsActive()){
            while(opModeIsActive()){
                float left_stick_y = gamepad1.left_stick_y;
                float left_stick_x = gamepad1.left_stick_x;
                float right_stick_x = gamepad1.right_stick_x;
                //telemetry.addData("Test", true);
                telemetry.addData("left_stick_x", gamepad2.left_stick_x);

                initialize_direction();//sets up the direction for motors and servos
                Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Hand_Rotator_Servo.setPosition(Hand_Rotator_Servo.getPosition() + gamepad2.left_stick_x*0.01);
                //Hand_Servo.setPosition(Hand_Servo.getPosition());
                telemetry.addData("Hand Servo Position", Hand_Servo.getPosition());
                telemetry.addData("Hand Servo Position incorrect", Hand_Rotator_Servo.getPosition());
                telemetry.addData("direction", Hand_Servo.getDirection());
                telemetry.addData("arm position", Arm_Motor.getCurrentPosition());
                telemetry.addData("Dist To Move", dist_to_move);
                //Hand_Servo.setDirection(gamepad2.a ? Servo.Direction.REVERSE: Servo.Direction.FORWARD);
                //Hand_Servo.setPosition(gamepad2.a ? 0.65: 0);

                if (gamepad2.left_trigger > 0){
                      if (Hand_Servo.getPosition() > 0){
                          //Hand_Servo.setPosition(Hand_Servo.getPosition() - (gamepad2.left_trigger*0.001));
                          //Hand_Servo.setPosition(Hand_Servo.getPosition() - ((Math.round(gamepad2.left_trigger * Math.pow(10, 2))/Math.pow(10, 2))*0.001));
                          Hand_Servo.setPosition(0);
                      }
                } else if (gamepad2.right_trigger > 0){
                    if (Hand_Servo.getPosition() < 0.65){
                        //Hand_Servo.setPosition(Hand_Servo.getPosition() + (gamepad2.right_trigger*0.001));
                        //Hand_Servo.setPosition(Hand_Servo.getPosition() + ((Math.round(gamepad2.right_trigger * Math.pow(10, 2))/Math.pow(10, 2))*0.001));
                        Hand_Servo.setPosition(0.65);
                    }
                } else if (gamepad2.b) {
                    Hand_Servo.setPosition(0);
                }
                telemetry.addData("Hand Servo grip", Hand_Servo.getPosition());

                dist_to_move += gamepad2.left_stick_y*0.01;
                Arm_Motor.setPower(0.5);
                if (Arm_Motor.getTargetPosition() == Arm_Motor.getCurrentPosition()){
                    Arm_Motor.setTargetPosition(Arm_Motor.getCurrentPosition());
                    Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (dist_to_move >= 1){
                    dist_to_move = 0;
                    Arm_Motor.setPower(0.5);
                    Arm_Motor.setTargetPosition(Arm_Motor.getCurrentPosition() + 1);
                    Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if(dist_to_move <= -1) {
                    dist_to_move = 0;
                    Arm_Motor.setPower(0.5);
                    Arm_Motor.setTargetPosition(Arm_Motor.getCurrentPosition() -1);
                    Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else {
                    Arm_Motor.setPower(0.00001);
                }
                Arm_Motor.setTargetPosition(Arm_Motor.getCurrentPosition());
                //Hand_Rotator_Servo.setPosition(0.5);
                Back_Left_Wheel.setPower(((left_stick_y - left_stick_x)*-1) + right_stick_x);
                Back_Right_Wheel.setPower(((left_stick_y + left_stick_x)*-1) - right_stick_x);
                Front_Left_Wheel.setPower(((left_stick_y + left_stick_x)*-1) + right_stick_x);
                Front_Right_Wheel.setPower(((left_stick_y - left_stick_x)*-1) - right_stick_x);
                //update loop
                telemetry.update();
            }
            //DataUpdate.active = false;
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
    private void display_data(){
        //create sections
        //1. directions 2. servo power 3. servo positions
    }
}
