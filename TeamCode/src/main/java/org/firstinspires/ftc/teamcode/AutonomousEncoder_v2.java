package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.RobotHardware.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.RobotHardware.DRIVE_SPEED;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels, otherwise you would use: RobotAutoDriveByTime;
 *
 * This code ALSO requires that the drive Motors have been configured such that a positive power command moves them
 * forward, and causes the encoders to count UP.
 *
 * The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 * The code is written using a method called: encoderDrive(speed, leftFrontInches, rightBackInches, rightFrontInches,
 * rightBackInches, timeoutS)
 *
 * that performs the actual movement.
 * This method assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Encoder v2", group="Robot")

public class AutonomousEncoder_v2 extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot" to access this class.
    RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() {
        double speed =0;
        double leftFrontDrive = 0;
        double leftBackDrive = 0;
        double rightFrontDrive = 0;
        double rightBackDrive = 0;
        double drive = 0;
        double turn = 0;
        double strafe = 0;
        double arm = 0;
        double handOffset = 0;

        // Initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        //Send a telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d", leftFrontDrive,leftBackDrive,
                rightFrontDrive, rightBackDrive);
        telemetry.update();


        // Wait for the game to start (the driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            robot.encoderDrive(DRIVE_SPEED, 30, 30, 30, 30);
            sleep(250);
            robot.encoderDrive(DRIVE_SPEED, -25, -25, -25, -25);
            sleep(250);
            robot.encoderDrive(DRIVE_SPEED, 39.5, -39.5, -39.5, 39.5);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000); // pause to display final telemetry message.
        }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */

        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;

        // Display it for the driver.
        telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget,
                newLeftBackTarget, newRightBackTarget);
        telemetry.addData("Currently at",  " at %7d :%7d", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }sleep(250);   // optional pause after each move.
        }
    }
}
