package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive")
public class Drive extends OpMode
{
    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor RearRight;
    DcMotor RearLeft;
    enum DriveMode {
        Tank,
        Arcade
    }

    DriveMode driveMode;

    @Override
    public void init() {

        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        RearRight = hardwareMap.dcMotor.get("RearRight");
        RearLeft = hardwareMap.dcMotor.get("RearLeft");

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        RearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        stopDrive();
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper || gamepad1.dpad_right)
        {
            mecanum(-1);
        }
        else if(gamepad1.left_bumper || gamepad1.dpad_left)
        {
            mecanum(1);
        } else if (driveMode == DriveMode.Arcade){
            MoveYouStupidPieceOfGarageMOVEEEE(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        } else if (driveMode == DriveMode.Tank) {
            MoveYouStupidPieceOfGarageMOVEEEETANK(-gamepad1.right_stick_y, -gamepad1.left_stick_y);
        }

        if (gamepad1.x) {
            driveMode = driveMode == DriveMode.Tank ? DriveMode.Arcade : DriveMode.Tank;
        }
//a+
    }

    void MoveYouStupidPieceOfGarageMOVEEEE(double x, double y,double spin)
    {
        FrontRight.setPower(-x+y-spin);
        FrontLeft.setPower(x+y+spin);
        RearRight.setPower(x+y-spin);
        RearLeft.setPower(-x+y+spin);
    }
    void MoveYouStupidPieceOfGarageMOVEEEETANK(double YR, double YL)
    {
        FrontRight.setPower(YR);
        FrontLeft.setPower(YL);
        RearRight.setPower(YR);
        RearLeft.setPower(YL);
    }

    void mecanum(double power)
    {
        FrontRight.setPower(power);
        FrontLeft.setPower(-power);
        RearRight.setPower(-power);
        RearLeft.setPower(power);

    }

    void setDrivePowers(double RL, double FL, double RR, double FR) {
        FrontRight.setPower(FR);
        FrontLeft.setPower(FL);
        RearRight.setPower(RR);
        RearLeft.setPower(RL);

    }

    void stopDrive() {
        setDrivePowers(0, 0, 0, 0);
    }

}