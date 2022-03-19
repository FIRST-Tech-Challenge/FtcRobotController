/* FTC Team 7572 - Version 2.0 (12/10/2021)
*/
package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * TeleOp Full Control.
 */
@TeleOp(name="Teleop-Skunkworks", group="7592")
//@Disabled
public class TestGreatNewFunctionality extends LinearOpMode {
    boolean gamepad1_triangle_last,   gamepad1_triangle_now   = false;  // Capping arm score position
    boolean gamepad1_circle_last,     gamepad1_circle_now     = false;  // Duck motor control
    boolean gamepad1_cross_last,      gamepad1_cross_now      = false;  // Capping arm claw open/close
    boolean gamepad1_square_last,     gamepad1_square_now     = false;  // Capping arm collect/store positions
    boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;  // gamepad1.dpad_up used live/realtime
    boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;  //   (see processDpadDriveMode() below)
    boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
    boolean gamepad1_l_bumper_last,   gamepad1_l_bumper_now   = false;
    boolean gamepad1_r_bumper_last,   gamepad1_r_bumper_now   = false;

    boolean gamepad2_triangle_last,   gamepad2_triangle_now   = false;  //
    boolean gamepad2_circle_last,     gamepad2_circle_now     = false;  // Freight Arm (Transport height)
    boolean gamepad2_cross_last,      gamepad2_cross_now      = false;  // Freight Arm (Collect height)
    boolean gamepad2_square_last,     gamepad2_square_now     = false;  // Intake reverse
    boolean gamepad2_dpad_up_last,    gamepad2_dpad_up_now    = false;  // Freight Arm (Hub-Top)
    boolean gamepad2_dpad_down_last,  gamepad2_dpad_down_now  = false;  // Freight Arm (Hub-Bottom) 
    boolean gamepad2_dpad_left_last,  gamepad2_dpad_left_now  = false;  // Freight Arm (Hub-Middle)
    boolean gamepad2_dpad_right_last, gamepad2_dpad_right_now = false;  // Freight Arm (score FRONT)
    boolean gamepad2_l_bumper_last,   gamepad2_l_bumper_now   = false;  // sweeper (reverse)
    boolean gamepad2_r_bumper_last,   gamepad2_r_bumper_now   = false;  // box servo (dump)

    /* Declare OpMode members. */
    HardwareBothHubs robot = new HardwareBothHubs();

    boolean autoBarrierDrive = false;
    boolean startedBarrier   = false;
    int     flatcount = 0;
    double  tiltAngle0 = 0.0;

    double[] tiltAngles = new double[12];
    int      tiltAnglesIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean atPosition;
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap,false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("State", "Running");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Refresh gamepad button status
            captureGamepad1Buttons();
            captureGamepad2Buttons();

            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();
            robot.headingIMU();

            if( processDriveOverBarrier() ) {
                // stop and wait for it to settle
                robot.stopMotion();
            }

            if( gamepad1_dpad_up_now && !gamepad1_dpad_up_last ) {
                startDriveOverBarrier( 0.40 );
            }

            if( gamepad1_dpad_down_now && !gamepad1_dpad_down_last ) {
                startDriveOverBarrier( -0.40 );
            }

            if( gamepad1_cross_now && !gamepad1_cross_last ) {
                robot.stopMotion();
                autoBarrierDrive = false;
            }

            // Hopefully this does everything capping arm.
            telemetry.addData( "robot.tiltAngle:", robot.tiltAngle );
            telemetry.addData( "autoBarrierDrive:", autoBarrierDrive );
            telemetry.addData( "tiltAngle0:", tiltAngle0 );
            telemetry.addData( "flatcount:", flatcount );
            telemetry.addData( "startedBarrier:", startedBarrier );

            for( int i=0; i<12; i++ )
                telemetry.addData( "tilt error", "%d = %.2f", i, tiltAngles[i] );
            telemetry.update();
        }
    } // runOpMode

    void startDriveOverBarrier( double motorPower ) {
        tiltAngle0 = robot.tiltAngle;
        robot.driveTrainMotors(motorPower, motorPower, motorPower, motorPower);
        autoBarrierDrive = true;
        startedBarrier   = false;
        flatcount = 0;
        tiltAnglesIndex = 0;
    } // startDriveOverBarrier

    boolean processDriveOverBarrier() {
        boolean fullyOverBarrier = false;

        // Are we in the middle of an auto-drive?
        if( !autoBarrierDrive ) return false;

        boolean flatAngle = (robot.tiltAngle > -0.75) && (robot.tiltAngle < 3.5);

        // Shift these through our last-12 array
        for( int i=0; i<11; i++)
            tiltAngles[i] = tiltAngles[i+1];
        tiltAngles[11] = robot.tiltAngle;

            // Are we done?
        if( startedBarrier ) {
            if( flatAngle) {
                if( ++flatcount >= 3 ) {
                    fullyOverBarrier = true;
                    autoBarrierDrive = false;
                }
            }
            else {
                flatcount = 0;
            }
        }
        else {
            if( robot.tiltAngle < -3.0 ) {
                startedBarrier = true;
            }
        }

        return fullyOverBarrier;
    } // processDriveOverBarrier

    /*---------------------------------------------------------------------------------*/
    void captureGamepad1Buttons() {
        gamepad1_triangle_last   = gamepad1_triangle_now;    gamepad1_triangle_now   = gamepad1.triangle;
        gamepad1_circle_last     = gamepad1_circle_now;      gamepad1_circle_now     = gamepad1.circle;
        gamepad1_cross_last      = gamepad1_cross_now;       gamepad1_cross_now      = gamepad1.cross;
        gamepad1_square_last     = gamepad1_square_now;      gamepad1_square_now     = gamepad1.square;
        gamepad1_dpad_up_last    = gamepad1_dpad_up_now;     gamepad1_dpad_up_now    = gamepad1.dpad_up;
        gamepad1_dpad_down_last  = gamepad1_dpad_down_now;   gamepad1_dpad_down_now  = gamepad1.dpad_down;
        gamepad1_dpad_left_last  = gamepad1_dpad_left_now;   gamepad1_dpad_left_now  = gamepad1.dpad_left;
        gamepad1_dpad_right_last = gamepad1_dpad_right_now;  gamepad1_dpad_right_now = gamepad1.dpad_right;
        gamepad1_l_bumper_last   = gamepad1_l_bumper_now;    gamepad1_l_bumper_now   = gamepad1.left_bumper;
        gamepad1_r_bumper_last   = gamepad1_r_bumper_now;    gamepad1_r_bumper_now   = gamepad1.right_bumper;
    } // captureGamepad1Buttons

    /*---------------------------------------------------------------------------------*/
    void captureGamepad2Buttons() {
        gamepad2_triangle_last   = gamepad2_triangle_now;    gamepad2_triangle_now   = gamepad2.triangle;
        gamepad2_circle_last     = gamepad2_circle_now;      gamepad2_circle_now     = gamepad2.circle;
        gamepad2_cross_last      = gamepad2_cross_now;       gamepad2_cross_now      = gamepad2.cross;
        gamepad2_square_last     = gamepad2_square_now;      gamepad2_square_now     = gamepad2.square;
        gamepad2_dpad_up_last    = gamepad2_dpad_up_now;     gamepad2_dpad_up_now    = gamepad2.dpad_up;
        gamepad2_dpad_down_last  = gamepad2_dpad_down_now;   gamepad2_dpad_down_now  = gamepad2.dpad_down;
        gamepad2_dpad_left_last  = gamepad2_dpad_left_now;   gamepad2_dpad_left_now  = gamepad2.dpad_left;
        gamepad2_dpad_right_last = gamepad2_dpad_right_now;  gamepad2_dpad_right_now = gamepad2.dpad_right;
        gamepad2_l_bumper_last   = gamepad2_l_bumper_now;    gamepad2_l_bumper_now   = gamepad2.left_bumper;
        gamepad2_r_bumper_last   = gamepad2_r_bumper_now;    gamepad2_r_bumper_now   = gamepad2.right_bumper;
    } // captureGamepad2Buttons
} // TeleopBlue
