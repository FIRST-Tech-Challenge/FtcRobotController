/* FTC Team 7572 - Version 2.0 (12/10/2021)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    boolean autoDrive = false;
    boolean autoDriveLast = false;
    FileWriter log;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            log = new FileWriter("autopilotData.txt", true);
            telemetry.addData("State", "Initializing (please wait)");
            telemetry.update();

            // Initialize robot hardware
            robot.init(hardwareMap,false);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("State", "Ready");
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive())
            {
                // Refresh gamepad button status
                captureGamepad1Buttons();
                captureGamepad2Buttons();

                // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
                robot.readBulkData();
                robot.headingIMU();

                //===================================================================
                // Check for an OFF-to-ON toggle of the gamepad2 DPAD UP
                autoDriveLast = autoDrive;
                if( gamepad2_dpad_up_now && !gamepad2_dpad_up_last) {
                    if(autoDrive) {
                        autoDrive = false;
                        robot.stopMotion();
                    } else {
                        autoDrive = true;
                        double driveSpeed = 0.4;
                        robot.frontLeftMotor.setPower(driveSpeed);
                        robot.frontRightMotor.setPower(driveSpeed);
                        robot.rearLeftMotor.setPower(driveSpeed);
                        robot.rearRightMotor.setPower(driveSpeed);
                    }
                }

                if(autoDrive) {
                    // First time through
                    if(!autoDriveLast) {
                        log.write("Starting autopilot session\r\n");
                    }
                    log.write("IMU Z: " + robot.imuZAngle + " Y: " + robot.imuYAngle + " X: " + robot.imuXAngle);
                    log.write(" FL: " + robot.frontLeftMotorAmps + " FR: " + robot.frontLeftMotorAmps + " RL: " + robot.rearLeftMotorAmps + " RR: " + robot.rearRightMotorAmps);
                    log.write("\r\n");
                } else {
                    log.write("IMU Z: " + robot.imuZAngle + " Y: " + robot.imuYAngle + " X: " + robot.imuXAngle);
                    log.write(" FL: " + robot.frontLeftMotorAmps + " FR: " + robot.frontLeftMotorAmps + " RL: " + robot.rearLeftMotorAmps + " RR: " + robot.rearRightMotorAmps);
                    log.write("\r\n");

                    // Last time through
                    if(autoDriveLast) {
                        log.write("Ending autopilot session\r\n");
                    }
                }
            }
            log.close();
        } catch (IOException e) {
        }
    } // runOpMode
    /*---------------------------------------------------------------------------------*/
    void captureGamepad1Buttons() {
        gamepad1_triangle_last   = gamepad1_triangle_now;    gamepad1_triangle_now   = gamepad1.triangle;
        gamepad1_circle_last     = gamepad1_circle_now;      gamepad1_circle_now     = gamepad1.circle;
        gamepad1_cross_last      = gamepad1_cross_now;       gamepad1_cross_now      = gamepad1.cross;
        gamepad1_square_last     = gamepad1_square_now;      gamepad1_square_now     = gamepad1.square;
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
