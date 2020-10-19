package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain_v3;

@Autonomous(name="Auto Opmode #1", group="Test")

public class Opmode_1 extends BasicAutonomous {

    //Note: Subsystems are instantiated in the BasicAutonomous Class

    @Override
    public void runOpMode() {

        drivetrain.init(hardwareMap, telemetry);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        // Encoder rest is handled in the Drivetrain init in Drivetrain class

        // Calibrate gyro

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && drivetrain.imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", drivetrain.imu.getCalibrationStatus().toString());
        telemetry.update();

        /////////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        // This is currently set up or field coordinates NOT TELATIVE to the last move
        gyroDrive(DRIVE_SPEED, 80.0, 0.0, 5);    // Drive FWD 110 inches
        gyroTurn( TURN_SPEED, 90.0, 3);         // Turn  CCW to -45 Degrees
        gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }



}
