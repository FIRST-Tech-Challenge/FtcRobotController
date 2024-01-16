/* FTC Team 7572 - Version 1.0 (12/29/2023)
 */
package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * TeleOp Servo Test Program
 */
@TeleOp(name="Teleop-ServoTest", group="7592")
//@Disabled
public class TeleopServoTest extends LinearOpMode {
    boolean gamepad1_triangle_last,   gamepad1_triangle_now   = false;  //
    boolean gamepad1_circle_last,     gamepad1_circle_now     = false;  //
    boolean gamepad1_cross_last,      gamepad1_cross_now      = false;  //
    boolean gamepad1_square_last,     gamepad1_square_now     = false;  //
    boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;  // gamepad1.dpad_up used live/realtime
    boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;  //   (see processDpadDriveMode() below)
    boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
    boolean gamepad1_l_bumper_last,   gamepad1_l_bumper_now   = false;
    boolean gamepad1_r_bumper_last,   gamepad1_r_bumper_now   = false;
    boolean gamepad1_touchpad_last,   gamepad1_touchpad_now   = false;
    boolean gamepad1_l_trigger_last,  gamepad1_l_trigger_now  = false;
    boolean gamepad1_r_trigger_last,  gamepad1_r_trigger_now  = false;

    int    selectedServo = 0;  // 0=col, 1=push, 2=wrist, 3=finger1, 4=finger2
    double collPos, pushPos, wristPos, finger1Pos, finger2Pos;
    double stepSize = 0.01;
    long      nanoTimeCurr=0, nanoTimePrev=0;
    double    elapsedTime, elapsedHz;

    /* Declare OpMode members. */
    HardwarePixelbot robot = new HardwarePixelbot(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap,false);

        // Start each value at the initialization position
        collPos = robot.COLLECTOR_SERVO_STORED;
        pushPos = robot.PUSH_SERVO_INIT;
        wristPos = robot.WRIST_SERVO_INIT;
        finger1Pos = robot.FINGER1_SERVO_DROP;
        finger2Pos = robot.FINGER2_SERVO_DROP;

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

            // Bulk-refresh the Control/Expansion Hub device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();

            //================ Update telemetry with current state ================
            telemetry.addData("Use CROSS to toggle between servos", " " );
            switch( selectedServo ) { // 0=col, 1=push, 2=wrist, 3=finger1, 4=finger2
                case 0 :
                    telemetry.addData("SELECTED:", "CollectorServo" );
                    telemetry.addData("value", "%.3f", collPos);
                    break;
                case 1 :
                    telemetry.addData("SELECTED:", "PushServo" );
                    telemetry.addData("value", "%.3f", pushPos );
                    telemetry.addData("Push Servo Angle", "%.1f", robot.getPushServoAngle());
                    break;
                case 2 :
                    telemetry.addData("SELECTED:", "WristServo" );
                    telemetry.addData("value", "%.3f", wristPos );
                    telemetry.addData("Wrist Servo Angle", "%.1f",robot.getWristServoAngle());
                    break;
                case 3 :
                    telemetry.addData("SELECTED:", "Finger1Servo" );
                    telemetry.addData("value", "%.3f", finger1Pos );
                    telemetry.addData("Finger Servo1 Angle", "%.1f",robot.getFingerServo1Angle());
                    break;
                case 4 :
                    telemetry.addData("SELECTED:", "Finger2Servo" );
                    telemetry.addData("value", "%.3f", finger2Pos );
                    telemetry.addData("Finger Servo2 Angle", "%.1f",robot.getFingerServo2Angle());
                    break;
                default :
                    selectedServo = 0;
                    break;
            } // switch()


            //================ CROSS SWITCHES WHICH SERVO WE'RE CONTROLLING ================
            if( gamepad1_cross_now && !gamepad1_cross_last)
            {
                selectedServo += 1;
                if( selectedServo > 4 ) selectedServo = 0;
            } // cross

            //================ LEFT BUMPER DECREASES SERVO POSITION ================
            if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last)
            {
                switch( selectedServo ) { // 0=col, 1=push, 2=wrist, 3=finger1, 4=finger2
                    case 0 :
                        collPos -= stepSize;
                        if( collPos < 0.0 ) collPos = 0.0;
                        if( collPos > 1.0 ) collPos = 1.0;
                        robot.collectorServo.setPosition(collPos);
                        break;
                    case 1 :
                        pushPos -= stepSize;
                        if( pushPos < 0.0 ) pushPos = 0.0;
                        if( pushPos > 1.0 ) pushPos = 1.0;
                        robot.pushServo.setPosition(pushPos);
                        break;
                    case 2 :
                        wristPos -= stepSize;
                        if( wristPos < 0.0 ) wristPos = 0.0;
                        if( wristPos > 1.0 ) wristPos = 1.0;
                        robot.wristServo.setPosition(wristPos);
                        break;
                    case 3 :
                        finger1Pos -= stepSize;
                        if( finger1Pos < 0.0 ) finger1Pos = 0.0;
                        if( finger1Pos > 1.0 ) finger1Pos = 1.0;
                        robot.fingerServo1.setPosition(finger1Pos);
                        break;
                    case 4 :
                        finger2Pos -= stepSize;
                        if( finger2Pos < 0.0 ) finger2Pos = 0.0;
                        if( finger2Pos > 1.0 ) finger2Pos = 1.0;
                        robot.fingerServo2.setPosition(finger2Pos);
                        break;
                    default :
                        break;
                } // switch()
            } // left bumper

            //================ RIGHT BUMPER INCREASES SERVO POSITION ================
            else if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last)
            {
                switch( selectedServo ) { // 0=col, 1=push, 2=wrist, 3=finger1, 4=finger2
                    case 0 :
                        collPos += stepSize;
                        if( collPos < 0.0 ) collPos = 0.0;
                        if( collPos > 1.0 ) collPos = 1.0;
                        robot.collectorServo.setPosition(collPos);
                        break;
                    case 1 :
                        pushPos += stepSize;
                        if( pushPos < 0.0 ) pushPos = 0.0;
                        if( pushPos > 1.0 ) pushPos = 1.0;
                        robot.pushServo.setPosition(pushPos);
                        break;
                    case 2 :
                        wristPos += stepSize;
                        if( wristPos < 0.0 ) wristPos = 0.0;
                        if( wristPos > 1.0 ) wristPos = 1.0;
                        robot.wristServo.setPosition(wristPos);
                        break;
                    case 3 :
                        finger1Pos += stepSize;
                        if( finger1Pos < 0.0 ) finger1Pos = 0.0;
                        if( finger1Pos > 1.0 ) finger1Pos = 1.0;
                        robot.fingerServo1.setPosition(finger1Pos);
                        break;
                    case 4 :
                        finger2Pos += stepSize;
                        if( finger2Pos < 0.0 ) finger2Pos = 0.0;
                        if( finger2Pos > 1.0 ) finger2Pos = 1.0;
                        robot.fingerServo2.setPosition(finger2Pos);
                        break;
                    default :
                        break;
                } // switch()
            } // right bumper

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            elapsedTime  = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;   // msec
            elapsedHz    =  1000.0 / elapsedTime;

            // Update telemetry data
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", elapsedTime, elapsedHz );
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        } // opModeIsActive

    } // runOpMode

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
//      gamepad1_touchpad_last   = gamepad1_touchpad_now;    gamepad1_touchpad_now   = gamepad1.touchpad;
        gamepad1_l_trigger_last  = gamepad1_l_trigger_now;   gamepad1_l_trigger_now  = (gamepad1.left_trigger >= 0.5);
        gamepad1_r_trigger_last  = gamepad1_r_trigger_now;   gamepad1_r_trigger_now  = (gamepad1.right_trigger >= 0.5);
    } // captureGamepad1Buttons

} // TeleopServoTest
