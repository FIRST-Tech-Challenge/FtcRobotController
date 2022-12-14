package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.RotateMove;

@TeleOp(name="RotateMoveTest", group="TEST")
@Disabled


public class RotateMoveTest extends OpMode {

    public Move move;
    public RotateMove rotateMoveTest;
    private DcMotor leftMotor, rightMotor, backRightMotor, backLeftMotor;

    @Override
    public void init(){ 
        move = new Move(leftMotor, rightMotor, backRightMotor, backLeftMotor);
    leftMotor = hardwareMap.dcMotor.get("FL");
    rightMotor = hardwareMap.dcMotor.get("FR");
    backLeftMotor = hardwareMap.dcMotor.get("BL");
    backRightMotor = hardwareMap.dcMotor.get("BR");
    rotateMoveTest = new RotateMove(leftMotor, rightMotor, backRightMotor, backLeftMotor, hardwareMap.get(BNO055IMU.class, "imu"));

    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up && gamepad1.dpad_right) //merge in fata
        {
            //move.MoveFull(1);
            rotateMoveTest.MoveFull(1,0.5);
        }
        else if(gamepad1.dpad_down && gamepad1.dpad_left) //merge in spate
        {
            //move.MoveFull(2);
            rotateMoveTest.MoveFull(2, 0.5);
        }
        else if(gamepad1.dpad_left && gamepad1.dpad_up) //merge stanga
        {
            //move.MoveFull(3);
            rotateMoveTest.MoveFull(1, 0.5);
        }
        else if(gamepad1.dpad_right && gamepad1.dpad_down) // merge dreapta
        {
            //move.MoveFull(4);
            rotateMoveTest.MoveFull(2, 0.5);
        }
        /*else if(gamepad1.dpad_up) //merge in fata
        {
            //move.MoveFull(1);
            rotateMoveTest.MoveFull(1,0);
        }

        else if(gamepad1.dpad_down) //merge in spate
        {
            //move.MoveFull(2);
            rotateMoveTest.MoveFull(2, 0);
        }
        else if(gamepad1.dpad_left) //merge stanga
        {
            //move.MoveFull(3);
            rotateMoveTest.MoveFull(3, 0);
        }
        else if(gamepad1.dpad_right) // merge dreapta
        {
            //move.MoveFull(4);
            rotateMoveTest.MoveFull(4, 0);
        }*/
        else
        {
            move.MoveStop();
        }
        //if(!(gamepad1.dpad_up) && !(gamepad1.dpad_down) && !(gamepad1.dpad_left) && !(gamepad1.dpad_right))
    }

    public void CurveBy(int direction){
        switch (direction){
            //merge in fata
            case 1: // fata stanga = scad puterea motoarelor din stanga
                leftMotor.setPower(0.5);
                backLeftMotor.setPower(-0.5);
                rightMotor.setPower(-1);
                backRightMotor.setPower(1);
                break;
            case 2: //fata dreapta = scad puterea motoarelor din dreapta
                leftMotor.setPower(-0.5);
                backLeftMotor.setPower(0.5);
                rightMotor.setPower(1);
                backRightMotor.setPower(-1);
                break;

            //merge in spate
            case 3: //spate stanga = scad puterea motoarelor din stanga
                leftMotor.setPower(0.5);
                backLeftMotor.setPower(-0.5);
                rightMotor.setPower(-1);
                backRightMotor.setPower(1);
                break;
            case 4: //spate dreapta = scad puterea motoarelor din dreapta
                leftMotor.setPower(-0.5);
                backLeftMotor.setPower(0.5);
                rightMotor.setPower(1);
                backRightMotor.setPower(-1);
        }
    }
}
