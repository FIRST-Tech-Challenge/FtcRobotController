package org.firstinspires.ftc.teamcode.Vivian;

import android.util.Log;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp

public class VivianAuto extends LinearOpMode {
    public void encoderMove(int inches){ //move the robot about 12 inches and back

    DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
    DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
    DcMotor fRight = hardwareMap.dcMotor.get("fRight");
    DcMotor bRight = hardwareMap.dcMotor.get("bRight");

    fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    fRight.setDirection(DcMotorSimple.Direction.REVERSE);
    bRight.setDirection(DcMotorSimple.Direction.FORWARD);

    fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fLeft.getCurrentPosition();

    double ticksPerInch = (537.7 / 1.4) / 11.87373601322835;//how many ticks in 1 inch


    while (Math.abs(fLeft.getCurrentPosition()) <= ticksPerInch * Math.abs(inches) && opModeIsActive()) {
        //debug
        telemetry.addData("Current Position", fLeft.getCurrentPosition());
        telemetry.addData("Target Position", ticksPerInch * Math.abs(inches));
        telemetry.update();


        if (Math.abs(fLeft.getCurrentPosition()) >= ticksPerInch * Math.abs(inches)) {
            fLeft.setPower(0);
            bLeft.setPower(0);
            fRight.setPower(0);
            bRight.setPower(0);
            //if the current position is greater then 12 inches, set power to 0
        }

        else { // same as : else if (ticksPerInch * Math.abs(inches) > fLeft.getCurrentPosition())

            if (inches > 0){
                fLeft.setPower(0.25);
                bLeft.setPower(0.25);
                fRight.setPower(0.25);
                bRight.setPower(0.25);
                // if amount of ticks is positive then set power to 0.25
            }
            if (inches < 0 ){
                fLeft.setPower(-0.25);
                bLeft.setPower(-0.25);
                fRight.setPower(-0.25);
                bRight.setPower(-0.25);
                //if the ticks are negative then set power to -0.25
            }

        }

    }

}
    public void proportionalMove (int inches){ //move robot 12 inches and back with P part

        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.getCurrentPosition();


        double ticksPerInch = (537.7 / 1.4) / 11.87373601322835;//how many ticks in 1 inch
        double error = ticksPerInch * inches;
        double P = 0.0008; // p constant

        telemetry.addData("error", error);
        telemetry.update();

        while (opModeIsActive() && Math.abs(error) > 35){
            error = ticksPerInch * inches - fLeft.getCurrentPosition();

            telemetry.addData("motor power", error * P);
            telemetry.update();

            fLeft.setPower(error * P);
            bLeft.setPower(error * P);
            fRight.setPower(error * P);
            bRight.setPower(error * P);

        }
    }



    public void proportionalTurn (double degrees){ //turn 90 degrees with P part and imu

        //set up motors

        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.getCurrentPosition();


        // imu set up
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters= new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
        imu.resetYaw();


        double error = degrees;
        double P = 0.008; // p constant

        telemetry.addData("error", error);
        telemetry.update();

        while (opModeIsActive() && Math.abs(error) > 2){
            double currentYaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);//current angle

            error = degrees - currentYaw;

            telemetry.addData("angle", currentYaw); // telemetry out current angle
            telemetry.addData("motor power", error * P); //telemetry out motor power

            fLeft.setPower(error * P);
            bLeft.setPower(error * P);
            fRight.setPower(-error * P);
            bRight.setPower(-error * P);

            telemetry.update();

        }
    }

    @Override
    public void runOpMode () throws InterruptedException {

        waitForStart();

        proportionalTurn(90); //turns 90 degrees

        //proportionalMove(12); move forward exactly 12 inches
        //proportionalMove(-12); move backwards exactly 12 inches

        //encoderMove(12); //move forward about 12 inches
        //encoderMove(-12); //move backwards about 12 inches

    }


}
