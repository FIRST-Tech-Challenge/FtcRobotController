package org.firstinspires.ftc.teamcode.wheelEncoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;


//motor name is NEVEREST ORBITAL 20 GEARMOTOR (AM-3637)
//537.6 clicks per revolution
//each revolution is 38 mm in diameter(1.49606 inches)
//537.6 clicks = 38mm(1.49606 inches)


public class Encoder extends LinearOpMode {

    DcMotor frontRight, frontLeft, backRight, backLeft;

    @Override public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        /*frontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        frontRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        backLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        backRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);*/

        waitForStart();



    }

    //sets power of each motor
    public void setPower(int frontRightPower, int backRightPower,int frontLeftPower,int backLeftPower)
    {
        frontRight.setTargetPosition(frontRightPower);
        frontLeft.setTargetPosition(backRightPower);
        backLeft.setTargetPosition(frontLeftPower);
        backRight.setTargetPosition(backLeftPower);
    }

    //stops driving
    public void stopDriving()
    {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}

