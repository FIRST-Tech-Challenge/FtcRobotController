package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Test.BasicAutonomous;

@Autonomous(name="Red Right Line Auto #1", group="Auto")

//////////////////////////////////////////////////////////
// BasicAutonomous contains the main methods and are extend here
// to keep the code more manageable.
/////////////////////////////////////////////////////////



public class RED_Right_Line_Auto_1 extends BasicAutonomous {
    /* Declare OpMode members. */
    // All of this comes from the BasicAutonomous Class

    @Override
    public void runOpMode() {

        drivetrain.init(hardwareMap);

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
        // This is currently set up or field coordinates NOT RELATIVE to the last move
        gyroDrive(DRIVE_SPEED, 60.0, 0.0, 5);    // Drive FWD 110 inches
        //gyroTurn( TURN_SPEED, 90.0, 3);         // Turn  CCW to -45 Degrees
        //gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


}
