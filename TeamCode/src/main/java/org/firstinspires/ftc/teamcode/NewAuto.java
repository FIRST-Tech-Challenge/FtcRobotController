package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.DriveMainAuto;

import static org.firstinspires.ftc.teamcode.CheckDriveStraight.isWithinTolerance;
import static org.firstinspires.ftc.teamcode.CheckDriveStraight.turnToCorrectSide;

@Autonomous
public class NewAuto extends LinearOpMode implements DriveMainAuto {

    public double rotationLeft = 0;
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //send data to console
        telemetry.addData("Status", "Initialized");

        //load motors
        loadMotors(hardwareMap, new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        frontRightDrive.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //send update data
        telemetry.update();
        waitForStart();

        int target = 180;
        while(facing(target)){
            telemetry.addData("hello", "1");
            telemetry.addData("hello222", rotationLeft);
            telemetry.addData("cur", (int)imu.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180);
            telemetry.addData("frontLeftDrive", frontLeftDrive.getMotor().getPower());
            telemetry.addData("frontRightDrive", frontRightDrive.getMotor().getPower());
            telemetry.addData("backRightDrive", backRightDrive.getMotor().getPower());
            telemetry.addData("backLeftDrive", backLeftDrive.getMotor().getPower());


            telemetry.update();
            drive(getRotationLeft(target));
        }
        rotationLeft=0;
        drive(getRotationLeft(1));
        waitMe(1, runtime);
//
//        while(facing(45)){
//            telemetry.addData("hello", "2");
//            telemetry.addData("hello222", rotationLeft);
//            telemetry.addData("cur", (int)imu.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180);
//            telemetry.addData("frontLeftDrive", frontLeftDrive.getMotor().getPower());
//            telemetry.addData("frontRightDrive", frontRightDrive.getMotor().getPower());
//            telemetry.addData("backRightDrive", backRightDrive.getMotor().getPower());
//            telemetry.addData("backLeftDrive", backLeftDrive.getMotor().getPower());
//
//
//            telemetry.update();
//            drive();
//        }
//        rotationLeft=0;
//        drive();
//        waitt(1, runtime);
//
//
//        while(facing(270)){
//            telemetry.addData("hello", "3");
//            telemetry.addData("hello222", rotationLeft);
//            telemetry.addData("cur", (int)imu.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180);
//            telemetry.addData("frontLeftDrive", frontLeftDrive.getMotor().getPower());
//            telemetry.addData("frontRightDrive", frontRightDrive.getMotor().getPower());
//            telemetry.addData("backRightDrive", backRightDrive.getMotor().getPower());
//            telemetry.addData("backLeftDrive", backLeftDrive.getMotor().getPower());
//
//
//            telemetry.update();
//            drive();
//        }
//        rotationLeft=0;
//        drive();
//        waitt(1, runtime);
//
//
//
//        while(facing(0)){
//            drive();
//        }

    }
    public boolean facing(int target){
        rotationLeft=0;
        boolean correctDriction=false;
        double power = 0.05;
        int cur = (int)imu.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180;
        if (!isWithinTolerance(cur, target, 3)){
            rotationLeft+=power/4;
            correctDriction=true;
        }
        if (!isWithinTolerance(cur, target, 20)){
            rotationLeft+=power/4;
        }
        if (!isWithinTolerance(cur, target, 40)){
            rotationLeft+=power/4;
        }
        if (!isWithinTolerance(cur, target, 80)){
            rotationLeft+=power/4;
        }
        return correctDriction;
    }

    public double getRotationLeft(int target) {
        return rotationLeft*(turnToCorrectSide(imu.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180, target) ? 1 : -1);


    }

    public void drive(double rl) {
        backLeftDrive.setPower(rl); //backR
        backRightDrive.setPower(-rl); //frontL
        frontLeftDrive.setPower(+rl);  //frontR
        frontRightDrive.setPower(-rl);
    }
    private void waitMe(double sec, ElapsedTime runtime){
        runtime.reset();
        while (runtime.seconds() < sec) {
        }
    }



}



