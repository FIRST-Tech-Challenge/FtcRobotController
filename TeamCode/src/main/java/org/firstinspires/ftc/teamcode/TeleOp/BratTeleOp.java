package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import   com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.BratTest;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
@TeleOp(name = "TeleOp Noaptea Cercetatorilor",group="TEST")
@Disabled
public class BratTeleOp extends OpMode {


    DcMotor armMotor, leftMotor, rightMotor, rightMotorBack, leftMotorBack;
    Move move;
    Rotate rotate;
    BratTest bratTest;

    @Override
    public void init() {
        armMotor = hardwareMap.dcMotor.get("AM");
        bratTest = new BratTest(armMotor);
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        if (gamepad2.right_stick_y < 0) {
            bratTest.Start(gamepad2.right_stick_y);
        }
        if (gamepad2.right_stick_y > 0) {
            bratTest.Start(gamepad2.right_stick_y);
        }
        else if (gamepad2.right_stick_y == 0) bratTest.Stop();

        if(gamepad1.left_stick_x!=0 || gamepad1.left_stick_y!=0){
            if(Math.abs(gamepad1.left_stick_x)>=Math.abs(gamepad1.left_stick_y)){
                rotate.RotateRaw(1, gamepad1.left_stick_x);
            }
//            else{
//                move.MoveRaw(1, gamepad1.left_stick_y);
//            }
        }
        else if(gamepad1.dpad_up) //merge in fata
        {
            //move.MoveFull(2);
            //move.MoveRaw(2,speed);


        }
        else if(gamepad1.dpad_down) //merge in spate
        {

            //move.MoveFull(1);

        }
        else if(gamepad1.dpad_left) //merge stanga
        {
            rotate.RotateFull(2);

        }
        else if(gamepad1.dpad_right) // merge dreapta
        {
            rotate.RotateFull(1);
        }
        //miscarea stanga-dreapta:
        else if(gamepad1.right_bumper)
        {

            move.MoveRaw(3, 0.5);
        }
        else if(gamepad1.left_bumper)
        {
            move.MoveRaw(4, 0.5);


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
        else if(gamepad2.right_stick_y==0) bratTest.Stop();

    }
}
