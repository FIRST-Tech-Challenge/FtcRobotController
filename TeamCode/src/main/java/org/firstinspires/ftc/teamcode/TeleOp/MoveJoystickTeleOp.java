package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Functions.Move;

@TeleOp(name="TestJoystick", group="TEST")

public class MoveJoystickTeleOp extends OpMode {
//    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
//
//    private Move move;

    @Override
    public void init() {
//        leftMotor = hardwareMap.dcMotor.get("FL");
//        rightMotor = hardwareMap.dcMotor.get("FR");
//        leftMotorBack = hardwareMap.dcMotor.get("BL");
//        rightMotorBack = hardwareMap.dcMotor.get("BR");
//
//
//        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
//        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
            telemetry.addLine("gamepad1.left_stick_x:" + gamepad1.left_stick_x);
            telemetry.addLine("\ngamepad1.left_stick_y:" + gamepad1.left_stick_y);
            telemetry.update();


            
        }
    }
}



//    @Override
//    public void loop() {
//
////        if(gamepad1.right_stick_x!=0 || gamepad1.right_stick_y!=0)
////        {
////            moveDirectJoystick.MoveSimple(-gamepad1.right_stick_x, +gamepad1.right_stick_y);
////        }
//        if(gamepad1.left_stick_x!=0 || gamepad1.left_stick_y!=0){
//            if(Math.abs(gamepad1.left_stick_x)>=Math.abs(gamepad1.left_stick_y)){
//
//            }
////            else{
////                move.MoveRaw(1, gamepad1.left_stick_y);
////            }
//        }





//        else{
//            move.MoveStop();
//        }
//        if(gamepad2.right_stick_y>0)
//        {
//            arm.Start(gamepad2.right_stick_y);
//        }
//        if(gamepad2.right_stick_y<0)
//        {
//            arm.Start(gamepad2.right_stick_y);
//        }
//        else if(gamepad2.right_stick_y==0) arm.Stop();



