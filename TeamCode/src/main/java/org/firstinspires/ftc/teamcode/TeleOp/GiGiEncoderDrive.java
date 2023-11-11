package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "GiGiEncoderDrive")
public class GiGiEncoderDrive extends LinearOpMode {
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor Arm;
    private DcMotor Linear;
    private CRServo Claw;
    private CRServo Climber;
    private DcMotor Spool;
    private Servo shoot;
    @Override
    public void runOpMode() {
        BLeft = hardwareMap.dcMotor.get("BLeft");
        BRight = hardwareMap.dcMotor.get("BRight");
        FLeft  = hardwareMap.dcMotor.get("FLeft");
        FRight = hardwareMap.dcMotor.get("FRight");
        Arm = hardwareMap.dcMotor.get("Arm");
        Linear = hardwareMap.dcMotor.get("Linear");
        Claw = hardwareMap.crservo.get("Claw");
        Climber = hardwareMap.crservo.get("Climber");
        Spool = hardwareMap.dcMotor.get("Spool");
        shoot = hardwareMap.servo.get("shoot");

        FRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Linear.setDirection(DcMotorSimple.Direction.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                drive();
                armMethod();
                servoMethod();
                climbermethod();
                shootermethod();
            }
        }
    }
    public void climbermethod(){
        if (gamepad2.dpad_left){
            Climber.setPower(1);
        }else if (gamepad2.dpad_up){
            Spool.setPower(1);
        }
        else if(gamepad2.dpad_right){
            Climber.setPower(-1);
        }else if(gamepad2.dpad_down){
            Spool.setPower(-1);
        }
        else{
            Spool.setPower(0);
            Climber.setPower(0);
        }
    }

    public void shootermethod(){
        if (gamepad1.a){
            shoot.setPosition(1);
        } else if (gamepad1.b) {
            shoot.setPosition(0.5);
        } else if (gamepad1.x){
            shoot.setPosition(0.3);
        } else if (gamepad1.y) {
            shoot.setPosition(0);
        }
    }


    public void drive() {
        if (gamepad1.right_stick_y != 0.0) {
            BLeft.setPower(0.5 * gamepad1.right_stick_y);
            BRight.setPower(0.5 * gamepad1.right_stick_y);
            FRight.setPower(0.5 * gamepad1.right_stick_y);
            FLeft.setPower(0.5 * gamepad1.right_stick_y);
        }
        // Strafing
        else if (gamepad1.right_stick_x != 0.0) {
            BLeft.setPower(0.5 * gamepad1.right_stick_x);
            BRight.setPower(-0.5 * gamepad1.right_stick_x);
            FRight.setPower(0.5 * gamepad1.right_stick_x);
            FLeft.setPower(-0.5 * gamepad1.right_stick_x);

        }
        // Rotation
        else if (gamepad1.left_stick_x != 0.0) {
            BLeft.setPower(-0.5 * gamepad1.left_stick_x);
            BRight.setPower(0.5 * gamepad1.left_stick_x);
            FRight.setPower(0.5 * gamepad1.left_stick_x);
            FLeft.setPower(-0.5 * gamepad1.left_stick_x);
        } else {
            BLeft.setPower(0);
            BRight.setPower(0);
            FRight.setPower(0);
            FLeft.setPower(0);
        }
    }
    public void armMethod(){
        if (gamepad2.right_stick_y != 0.0) {
            Linear.setPower(0.5 * gamepad2.right_stick_y);

        }else if(gamepad2.left_stick_y != 0.0){
            Arm.setPower(0.5 * gamepad2.left_stick_y);
        }else {
            Arm.setPower(0);
            Linear.setPower(0);
        }
    }
    public void servoMethod(){
        if (gamepad2.right_bumper){
            Claw.setPower(1);
        }else if(gamepad2.left_bumper){
            Claw.setPower(-1);
        }
        else{
            Claw.setPower(0);
        }
    }
}