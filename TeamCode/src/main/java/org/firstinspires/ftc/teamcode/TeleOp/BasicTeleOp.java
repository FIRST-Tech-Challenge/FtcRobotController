package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;

@TeleOp(name="Basic TeleOp", group="GAME")
public class BasicTeleOp extends OpMode {
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private DcMotor armMotor;
    private Servo leftServo, rightServo;
    private Move move;
    private Rotate rotate;
    private ClawServos clawServos;
    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        armMotor = hardwareMap.dcMotor.get("AM");
        leftServo = hardwareMap.servo.get("LS");
        rightServo = hardwareMap.servo.get("RS");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);


        clawServos = new ClawServos(leftServo, rightServo);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_x!=0 || gamepad1.left_stick_y!=0){
            if(Math.abs(gamepad1.left_stick_x)>=Math.abs(gamepad1.left_stick_y)){
                rotate.RotateRaw(1, gamepad1.left_stick_x);
            }
            else{
                move.MoveRaw(1, gamepad1.left_stick_y);
            }
        }
//        else if(gamepad1.dpad_up)
//        {
//            move.MoveFull(2);
//        }
//        else if(gamepad1.dpad_down)
//        {
//            move.MoveFull(1);
//        }
        else if(gamepad1.dpad_left)
        {
            rotate.RotateFull(2);
        }
        else if(gamepad1.dpad_right)
        {
            rotate.RotateFull(1);
        }
        else if(gamepad1.right_bumper)
        {
            move.MoveRaw(4, 0.5);
        }
        else if(gamepad1.left_bumper)
        {
            move.MoveRaw(3, 0.5);
        }
        else{
            move.MoveStop();
        }

        if(gamepad1.right_trigger>0){
            move.MoveRaw(2,gamepad1.right_trigger);
        }
        if(gamepad1.left_trigger>0){
            move.MoveRaw(1,gamepad1.left_trigger);
        }
        if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
            move.MoveStop();
        }
        if(gamepad1.x)
        {
           // clawServos.SwitchAndWait(1,getRuntime());
            leftServo.setPosition(1);
            rightServo.setPosition(0);
        }
        if(gamepad1.a)
        {
            leftServo.setPosition(0);
            rightServo.setPosition(1);
        }
        if(gamepad1.right_stick_y!=0)
        {
            armMotor.setPower(gamepad1.right_stick_y);
        }
        if(gamepad1.right_stick_y==0)
        {
            armMotor.setPower(0);
        }
    }
}
