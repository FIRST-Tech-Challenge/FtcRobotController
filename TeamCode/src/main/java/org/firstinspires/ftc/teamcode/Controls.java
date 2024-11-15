package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class Controls extends LinearOpMode {
    private DcMotor Front_Left_Wheel;
    private DcMotor Back_Left_Wheel;
    private DcMotor Front_Right_Wheel;
    private DcMotor Back_Right_Wheel;
    private DcMotor Arm_Rotator;
    private CRServo Arm_Extender_Servo;
    private Servo Hand_Servo;
    private ElapsedTime myElapsedTime;
    @Override
    public void runOpMode() {
        //setup for servos
        Front_Left_Wheel = hardwareMap.get(DcMotor.class, "Front_Left_Wheel");
        Back_Left_Wheel = hardwareMap.get(DcMotor.class, "Back_Left_Wheel");
        Front_Right_Wheel = hardwareMap.get(DcMotor.class, "Front_Right_Wheel");
        Back_Right_Wheel = hardwareMap.get(DcMotor.class, "Back_Right_Wheel");
        Arm_Rotator = hardwareMap.get(DcMotor.class, "arm_rotator");
        Arm_Extender_Servo = hardwareMap.get(CRServo.class, "cont_rot_servo");
        //Arm_Rotator_Servo = hardwareMap.get(java.lang.Class, 171);
        //Arm_Extender_Servo = hardwareMap.get(CRServo.class, "Arm_Extender_Servo");
        Hand_Servo = hardwareMap.get(Servo.class, "Hand_Servo");
        //main loop
        waitForStart();

        if(opModeIsActive()) {
            while(opModeIsActive()) {
                movement_control_update();
                int x = 0;
                telemetry.addData("Something", x);
                telemetry.addLine("adfadfas");
                arm_control_update();
                telemetry.update();
            }
        }

    }
    //movement controls update
    public void movement_control_update() {
        float left_stick_x = gamepad1.left_stick_x * -1.6f;
        float left_stick_y = gamepad1.left_stick_y * (gamepad1.right_bumper ? 1f : 0.6f);
        float right_stick_x = gamepad1.right_stick_x;
        telemetry.addData("z speed", left_stick_y);
        telemetry.addData("gamepad1 right bumper active", gamepad1.right_bumper);
        telemetry.addData("x speed", left_stick_x);
        if(gamepad1.left_stick_y > 0) {
            Back_Left_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
            Back_Right_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
            Front_Left_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
            Front_Right_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
            /*
            Back_Left_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
            Back_Right_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
            Front_Left_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
            Front_Right_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
            */
        }
        Back_Left_Wheel.setPower(((left_stick_y - left_stick_x)*-1) + right_stick_x);
        Back_Right_Wheel.setPower(((left_stick_y + left_stick_x)*-1) - right_stick_x);
        Front_Left_Wheel.setPower(((left_stick_y + left_stick_x)*-1) + right_stick_x);
        Front_Right_Wheel.setPower(((left_stick_y - left_stick_x)*-1) - right_stick_x);
    }
    //arm controls update
    public void arm_control_update(){
        /*if(gamepad2.left_stick_y > 0) {
            Hand_Servo.setDirection(Servo.Direction.FORWARD);
            Hand_Servo.setPosition(gamepad2.left_stick_y);
            //Hand_Servo.setPower(gamepad2.right_trigger);


        } else if (gamepad2.right_stick_y < 0) {
            Hand_Servo.setDirection(Servo.Direction.REVERSE);
            //Hand_Servo.setPower(gamepad2.left_trigger);
            Hand_Servo.setPosition(gamepad2.left_stick_y);
        }*/


        telemetry.addData("right trigger", gamepad2.right_trigger);
        telemetry.addData("left trigger", gamepad2.left_trigger);
        telemetry.addData("Hand Servo Direction", Hand_Servo.getDirection());
        telemetry.addData("Hand Servo Position", Hand_Servo.getPosition());
        Arm_Rotator.setPower(gamepad2.left_stick_y*((gamepad2.left_stick_y > 0 ? 0.4 : 0.5) - (gamepad2.right_bumper ? 0.1f : 0)) *-1);
        telemetry.addData("arm power", Arm_Rotator.getPower());
        Arm_Rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("arm current rotation", Arm_Rotator.getCurrentPosition());
        telemetry.addData("left stick", gamepad2.left_stick_y);
        if (gamepad2.right_stick_y*-1 != 0) {
            Hand_Servo.setPosition(gamepad2.right_stick_y*-1);
        }

        if(gamepad2.x) {
            Arm_Extender_Servo.setDirection(DcMotorSimple.Direction.FORWARD);
            telemetry.addData("arm rotation direction", "Forward");
        } else if (gamepad2.y) {
            Arm_Extender_Servo.setDirection(DcMotorSimple.Direction.REVERSE);
            telemetry.addData("arm rotation direction", "Reverse");
        }
        if(gamepad2.x || gamepad2.y) {
            Arm_Extender_Servo.setPower(gamepad2.x ? bool_to_numb(gamepad2.x) : bool_to_numb(gamepad2.y));

        } else {
            Arm_Extender_Servo.setPower(0);
        }
        if(gamepad2.a) {
            Arm_Extender_Servo.setPower(0);
        }
    }
    public static double bool_to_numb(boolean item){
        return item ? 1 : 0;
    }

}