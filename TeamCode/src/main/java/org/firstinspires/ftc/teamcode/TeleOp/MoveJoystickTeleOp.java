package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.DistanceSensorFunction;
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
    private RotationDetector rotationDetector;
    private MoveJoystick moveJoystick;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");

        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotateMoveTest = new RotateMove(leftMotor, rightMotor, leftMotorBack, rightMotorBack, hardwareMap.get(BNO055IMU.class, "imu"));
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        moveJoystick = new MoveJoystick(leftMotor, rightMotor, leftMotorBack, rightMotorBack);


        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {

        double orientation = rotationDetector.ReturnPositiveRotation();

//        if ((int)orientation % 90 != 0) AutocorrectAngle(orientation);
//        orientation = rotationDetector.ReturnPositiveRotation();


        int direction = (int)Math.ceil(orientation / 90) % 4;


        if (gamepad1.right_bumper) AutocorrectAngle(orientation);



        if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
            if (Math.abs(gamepad1.right_stick_x) < Math.abs(gamepad1.right_stick_y)){
                if (gamepad1.right_stick_y < 0) moveJoystick.MoveJoystickRaw(direction,gamepad1.right_stick_y);
                else moveJoystick.MoveJoystickRaw((direction + 2) % 4,gamepad1.right_stick_y);


            }
            else if (Math.abs(gamepad1.right_stick_x) > Math.abs(gamepad1.right_stick_y)){
                if (gamepad1.right_stick_x > 0) moveJoystick.MoveJoystickRaw((direction + 1) % 4,gamepad1.right_stick_x);
                else moveJoystick.MoveJoystickRaw((direction + 3) % 4,gamepad1.right_stick_x);
            }
//            else{
//                if (gamepad1.right_stick_x > 0.3 && gamepad1.right_stick_y < -0.3) moveJoystick.JoystickCurveBy(direction);
//                if (gamepad1.right_stick_x < -0.3 && gamepad1.right_stick_y < -0.3) moveJoystick.JoystickCurveBy((direction + 1) % 4);
//                if (gamepad1.right_stick_x > 0.3 && gamepad1.right_stick_y > 0.3) moveJoystick.JoystickCurveBy((direction + 3) % 4);
//                if (gamepad1.right_stick_x < -0.3 && gamepad1.right_stick_y > 0.3) moveJoystick.JoystickCurveBy((direction + 2) % 4);
//            }


            telemetry.addLine("gamepad1.left_stick_x:" + gamepad1.left_stick_x);
            telemetry.addLine("gamepad1.left_stick_y:" + gamepad1.left_stick_y);
            telemetry.addLine("orientation:" + rotationDetector.ReturnPositiveRotation());
            telemetry.update();
        }
        else move.MoveStop();


        if (gamepad1.left_stick_x != 0){
            if (gamepad1.left_stick_x > 0) rotate.RotateRaw(2, gamepad1.left_stick_x);
            else rotate.RotateRaw(1, gamepad1.left_stick_x);
        } //alternativa pt rotatie




//        else move.MoveStop(); //posibil sa interfereze cu primul nest



    }










    private void AutocorrectAngle(double angle)
    {
        if ((int)angle % 90 != 0){
            if ((int)angle % 90 < 45)
                while ((int)angle % 90 != 0){
                    rotate.RotateRaw(1, 0.5);
                    angle = rotationDetector.ReturnPositiveRotation();
                }
            else
                while ((int)angle % 90 != 0){
                    rotate.RotateRaw(2, 0.5);
                    angle = rotationDetector.ReturnPositiveRotation();
                }
        }
        else return;
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



