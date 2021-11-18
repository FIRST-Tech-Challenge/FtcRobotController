package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "soooo")
@Disabled
public class Saaa extends LinearOpMode {
    MecanumChassis robot = new MecanumChassis();



    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition());

        telemetry.addData("IMU Mode", "IMU calibrating....");
        telemetry.update();

        //make sure the IMU gyro is calibrated before continue
        while (!isStopRequested() && !robot.imu.isGyroCalibrated() &&
                !robot.imu.isAccelerometerCalibrated() &&
                !robot.imu.isMagnetometerCalibrated() &&
                !robot.imu.isSystemCalibrated()) {
            idle();
        }
        telemetry.addData("IMU Mode", "IMU calibrating done");
        telemetry.update();

        waitForStart();


    }

    private void setPower(double frontLeft, double frontRight, double backLeft, double backRight){
        robot.leftFrontDrive.setPower(frontLeft);
        robot.rightFrontDrive.setPower(frontRight);
        robot.leftRearDrive.setPower(backLeft);
        robot.rightRearDrive.setPower(backRight);
    }

    private void arm(double power, int target, int margin){
        robot.arm.setTargetPosition(target);
        if(Math.abs(robot.arm.getCurrentPosition()-target) < margin){
            robot.arm.setPower(power);
        }
        robot.arm.setPower(0);
    }
}
