package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(group="chrisBot", name="PID Drive Test with Objects and Encoders")

public class ObjectPIDDriveEncoder extends LinearOpMode {
    chrisBot robot = new chrisBot();
    PID pidDrive;
    double power = 0.5;

    public void runOpMode() {
        robot.init(hardwareMap, telemetry, false, false);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        drive(12, 5000);
    }
    private void drive(double inches, int ms) {
        int FLTarget, FRTarget, BLTarget, BRTarget;
        ElapsedTime x = new ElapsedTime();
        pidDrive = new PID(0, new double[]{0.003, 0.0003, 0});


        // Determine new target position, and pass to motor controller
        int countsToTravel = (int)(inches * robot.COUNTS_PER_INCH);
        robot.motorFrontLeft.setTargetPosition(robot.motorFrontLeft.getCurrentPosition() + countsToTravel);
        robot.motorFrontRight.setTargetPosition(robot.motorFrontRight.getCurrentPosition() + countsToTravel);
        robot.motorBackLeft.setTargetPosition(robot.motorBackLeft.getCurrentPosition() + countsToTravel);
        robot.motorBackRight.setTargetPosition(robot.motorBackRight.getCurrentPosition() + countsToTravel);

        pidDrive.setActual(robot.getAngle());
        x.reset();

        // Turn On RUN_TO_POSITION
        robot.setAllDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // keep looping while we are still active, and there is time left and motors are running.
        while (robot.isBusy() && x.milliseconds() < ms) {
            double correction = pidDrive.calcCorrection();
            robot.setDrivePower(new double[]{power+correction,power-correction,power+correction,power-correction});
        }

        robot.setAllDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.setAllDrivePower(0);
    }



}
