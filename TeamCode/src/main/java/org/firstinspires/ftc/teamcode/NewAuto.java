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
        double ticks = 3 / straight.getInchesPerCount();
        int position = straight.getMotor().getCurrentPosition() + (int)ticks;
        straight.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        straight.getMotor().setTargetPosition(position);
        straight.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!isWithinTolerance(straight.getMotor().getCurrentPosition(), straight.getMotor().getTargetPosition(), 20)){
            frontRightDrive.setPower(-0.4);
            frontLeftDrive.setPower(-0.4);
            backLeftDrive.setPower(-0.4);
            backRightDrive.setPower(-0.4);
            frontRightDrive.setPower(UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
            backRightDrive.setPower(UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
            frontLeftDrive.setPower(UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
            backLeftDrive.setPower(UpdatePowerTypes.startEndUpdatePower(motorEx, CUR));
            telemetry.addData("tar", straight.getMotor().getTargetPosition());
            telemetry.addData("cur", straight.getMotor().getCurrentPosition());
            telemetry.addData("busy", straight.getMotor().isBusy());
            telemetry.addData("zero", !isWithinTolerance(straight.getMotor().getCurrentPosition(), straight.getMotor().getTargetPosition(), 10));

            telemetry.update();
        }
        stopMotors();


//        while(facing(90)){
//            telemetry.addData("hello", "1");
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
        if (!isWithinTolerance(cur, target, 5)){
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

    public void drive() {
        backLeftDrive.setPower(-rotationLeft); //backR
        backRightDrive.setPower(+rotationLeft); //frontL
        frontLeftDrive.setPower(-rotationLeft);  //frontR
        frontRightDrive.setPower(+rotationLeft);
        telemetry.addData("drive", gamepad1.left_stick_y);

    }

    private void waitt(double sec, ElapsedTime runtime){
        runtime.reset();
        while(runtime.seconds() < sec){

        }
    }

    public boolean hasReachedTarget(int startValue, int target, int tickIncrement) {
        int currentValue = startValue;

        // Check direction based on the relationship between startValue and target
        if (tickIncrement > 0) {
            // If increment is positive, loop until currentValue >= target
            while (currentValue < target) {
//                currentValue += tickIncrement;
                telemetry.addData("Current value: ", currentValue); // Print current value
            }
        } else if (tickIncrement < 0) {
            // If increment is negative, loop until currentValue <= target
            while (currentValue > target) {
//                currentValue += tickIncrement;
                telemetry.addData("Current value: ", currentValue); // Print current value
            }
        }

        // Return true because the loop only exits when the target is reached or surpassed
        return (tickIncrement > 0 && currentValue >= target) || (tickIncrement < 0 && currentValue <= target);
    }


}



