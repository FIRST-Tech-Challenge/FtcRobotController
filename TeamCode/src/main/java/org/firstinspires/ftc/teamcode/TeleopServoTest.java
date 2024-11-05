/* FTC Team 7572 - Version 1.0 (11/01/2024) */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    int     selectedServo = 0;  // 0=push, 1=wrist, 2=gecko
    double  elbowPos, wristPos;
    boolean geckoOn = false;
    double  stepSize = 0.01;
    long    nanoTimeCurr=0, nanoTimePrev=0;
    double  elapsedTime, elapsedHz;

    /* Declare OpMode members. */
//  Hardware2025Bot robot = new Hardware2025Bot(telemetry);
    Hardware2025Bot robot = new Hardware2025Bot();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap,false);

        // Start each value at the initialization position
        elbowPos = robot.ELBOW_SERVO_INIT;
        wristPos = robot.WRIST_SERVO_INIT;
        geckoOn = false;

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
            telemetry.addData("Use left/right BUMPERS to index servos", " " );
            switch( selectedServo ) { // 0=push, 1=wrist, 2=gecko
                case 0 :
                    telemetry.addData("SELECTED:", "ElbowServo" );
                    telemetry.addData("Elbow Servo Position", "%.3f", robot.getElbowServoPos() );
                    telemetry.addData("Elbow Servo Angle", "%.1f", robot.getElbowServoAngle());
                    break;
                case 1 :
                    telemetry.addData("SELECTED:", "WristServo" );
                    telemetry.addData("Wrist Servo Position", "%.3f", robot.getWristServoPos() );
                    telemetry.addData("Wrist Servo Angle", "%.1f",robot.getWristServoAngle());
                    break;
                case 2 :
                    telemetry.addData("SELECTED:", "IntakeServo" );
                    telemetry.addData("Intake Servo power", "%.1f", robot.geckoServo.getPower() );
                    break;
                default :
                    selectedServo = 0;
                    break;
            } // switch()

            //================ CROSS SWITCHES WHICH SERVO WE'RE CONTROLLING ================
            if( gamepad1_cross_now && !gamepad1_cross_last)
            {
                selectedServo += 1;
                if( selectedServo > 2 ) selectedServo = 0;
            } // cross

            //================ LEFT BUMPER DECREASES SERVO POSITION ================
            if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last)
            {
                switch( selectedServo ) { // 0=push, 1=wrist, 2=gecko
                    case 0 :
                        elbowPos -= stepSize;
                        if( elbowPos < 0.0 ) elbowPos = 0.0;
                        if( elbowPos > 1.0 ) elbowPos = 1.0;
                        robot.elbowServo.setPosition(elbowPos);
                        break;
                    case 1 :
                        wristPos -= stepSize;
                        if( wristPos < 0.0 ) wristPos = 0.0;
                        if( wristPos > 1.0 ) wristPos = 1.0;
                        robot.wristServo.setPosition(wristPos);
                        break;
                    case 2 :
                        if(geckoOn){
                            robot.geckoServo.setPower(0.0);
                            geckoOn = false;
                        }
                        else{
                            robot.geckoServo.setPower(-1.0);
                            geckoOn = true;
                        }
                        break;
                    default :
                        break;
                } // switch()
            } // left bumper

            //================ RIGHT BUMPER INCREASES SERVO POSITION ================
            else if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last)
            {
                switch( selectedServo ) { // 0=push, 1=wrist, 2=gecko
                    case 0 :
                        elbowPos += stepSize;
                        if( elbowPos < 0.0 ) elbowPos = 0.0;
                        if( elbowPos > 1.0 ) elbowPos = 1.0;
                        robot.elbowServo.setPosition(elbowPos);
                        break;
                    case 1 :
                        wristPos += stepSize;
                        if( wristPos < 0.0 ) wristPos = 0.0;
                        if( wristPos > 1.0 ) wristPos = 1.0;
                        robot.wristServo.setPosition(wristPos);
                        break;
                    case 2 :
                        if(geckoOn){
                            robot.geckoServo.setPower(0.0);
                            geckoOn = false;
                        }
                        else{
                            robot.geckoServo.setPower(1.0);
                            geckoOn = true;
                        }
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
