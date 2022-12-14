package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Functions.MoveJoystick;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Functions.Arm;
import org.firstinspires.ftc.teamcode.Functions.ArmServos;
import org.firstinspires.ftc.teamcode.Functions.Collector;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.MoveDirectJoystick;
import org.firstinspires.ftc.teamcode.Functions.MoveJoystick;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotateMove;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.Vacuum;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.RotateMove;

@TeleOp(name="TestJoystick", group="TEST")

public class MoveJoystickTeleOp extends OpMode {
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;

    private Move move;
    private Rotate rotate;
    public RotateMove rotateMoveTest;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");

        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotateMoveTest = new RotateMove(leftMotor, rightMotor, leftMotorBack, rightMotorBack, hardwareMap.get(BNO055IMU.class, "imu"));

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {

        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
            if (Math.abs(gamepad1.left_stick_x) < Math.abs(gamepad1.left_stick_y)){
                if (gamepad1.left_stick_y > 0) move.MoveRaw(1,gamepad1.left_stick_y);
                else move.MoveRaw(2,gamepad1.left_stick_y);

            }
            else if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)){
                if (gamepad1.left_stick_x > 0) move.MoveRaw(3,gamepad1.left_stick_x);
                else move.MoveRaw(4,gamepad1.left_stick_x);
            }
            else{
                if (gamepad1.left_stick_x < 0 && gamepad1.left_stick_y > 0) rotateMoveTest.CurveBy(1);
                if (gamepad1.left_stick_x > 0 && gamepad1.left_stick_y > 0) rotateMoveTest.CurveBy(2);
                if (gamepad1.left_stick_x < 0 && gamepad1.left_stick_y < 0) rotateMoveTest.CurveBy(3);
                if (gamepad1.left_stick_x > 0 && gamepad1.left_stick_y < 0) rotateMoveTest.CurveBy(4);
            }


            telemetry.addLine("gamepad1.left_stick_x:" + gamepad1.left_stick_x);
            telemetry.addLine("\ngamepad1.left_stick_y:" + gamepad1.left_stick_y);
            telemetry.update();
        }
        else move.MoveStop();


        if (gamepad1.right_stick_x != 0){
            if (gamepad1.right_stick_x > 0) rotate.RotateRaw(2, gamepad1.right_stick_x);
            else rotate.RotateRaw(1, gamepad1.right_stick_x);
        }
//        else move.MoveStop(); //posibil sa interfereze cu primul nest

    }
}




























//    @Override
//    public void loop() {
//        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
//            telemetry.addLine("gamepad1.left_stick_x:" + gamepad1.left_stick_x);
//            telemetry.addLine("\ngamepad1.left_stick_y:" + gamepad1.left_stick_y);
//            telemetry.update();
//
//
//
//        }
//    }


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



