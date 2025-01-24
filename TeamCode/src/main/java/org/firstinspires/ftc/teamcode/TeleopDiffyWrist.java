/* FTC Team 7572 - Version 1.0 (01/19/2025) */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Diffy Wrist Servo Test Program
 */
@TeleOp(name="Teleop-DiffyTest", group="Test")
@Disabled
public class TeleopDiffyWrist extends LinearOpMode {
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

    private Servo clawServo   = null;
    private Servo diffyLServo = null;
    private Servo diffyRServo = null;

    int     selectedServo = 1;  // 1=claw, 2=diffyL, 3=diffyR
    double  clawPos, diffyLPos, diffyRPos;
    double  stepSize = 0.010;
    long    nanoTimeCurr=0, nanoTimePrev=0;
    double  elapsedTime, elapsedHz;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware (autonomous=true initializes servos)
        clawServo   = hardwareMap.servo.get("claw");            // servo port 2 (Expansion Hub)
        diffyLServo = hardwareMap.servo.get("diffyL");          // servo port 0 (Expansion Hub)
        diffyRServo = hardwareMap.servo.get("diffyR");          // servo port 1 (Expansion Hub)

        // Preload each variable with the initialization position
        clawPos   = 0.5;  clawServo.setPosition(clawPos);
        diffyLPos = 0.5;  diffyLServo.setPosition(diffyLPos);
        diffyRPos = 0.5;  diffyRServo.setPosition(diffyRPos);

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
            clawPos   = clawServo.getPosition();
            diffyLPos = diffyLServo.getPosition();
            diffyRPos = diffyRServo.getPosition();

            //================ Update telemetry with current state ================
            telemetry.addData("Use CROSS to toggle between servos", " " );
            telemetry.addData("Use left/right BUMPERS to index servos", " " );
            switch( selectedServo ) { // 1=claw, 2=diffyL, 3=diffyR
                case 1 :
                    telemetry.addData("SELECTED:", "clawServo" );
                    telemetry.addData("Claw Servo Position", "%.3f", clawPos );
//                  telemetry.addData("Claw Servo Angle", "%.1f",    clawServo.getElbowServoAngle() );
                    break;
                case 2 :
                    telemetry.addData("SELECTED:", "diffyLServo" );
                    telemetry.addData("diffyLPos Servo Position", "%.3f", diffyLPos );
//                  telemetry.addData("diffyLPos Servo Angle", "%.1f",diffyLServo.getServoAngle());
                    break;
                case 3 :
                    telemetry.addData("SELECTED:", "diffyRServo" );
                    telemetry.addData("diffyRPos Servo Position", "%.3f", diffyRPos );
//                  telemetry.addData("diffyRPos Servo Angle", "%.1f",diffyRServo.getServoAngle());
                    break;
                default :
                    selectedServo = 1;
                    break;
            } // switch()

            //================ CROSS SWITCHES WHICH SERVO WE'RE CONTROLLING ================
            if( gamepad1_cross_now && !gamepad1_cross_last)
            {
                selectedServo += 1;
                if( selectedServo > 3 ) selectedServo = 1;
            } // cross

            if( gamepad1_triangle_now && !gamepad1_triangle_last)
            {
                clawServo.setPosition(0.65);
                diffyLServo.setPosition(0.5);
                diffyRServo.setPosition(0.5);
            } // triangle

            //================ LEFT BUMPER DECREASES SERVO POSITION ================
            if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last)
            {
                switch( selectedServo ) { // 1=claw, 2=diffyL, 3=diffyR
                    case 1 :
                        clawPos -= stepSize;
                        if( clawPos < 0.0 ) clawPos = 0.0;
                        if( clawPos > 1.0 ) clawPos = 1.0;
                        clawServo.setPosition(clawPos);
                        break;
                    case 2 :
                        diffyLPos -= stepSize;
                        if( diffyLPos < 0.0 ) diffyLPos = 0.0;
                        if( diffyLPos > 1.0 ) diffyLPos = 1.0;
                        diffyLServo.setPosition(diffyLPos);
                        break;
                    case 3 :
                        diffyRPos -= stepSize;
                        if( diffyRPos < 0.0 ) diffyRPos = 0.0;
                        if( diffyRPos > 1.0 ) diffyRPos = 1.0;
                        diffyRServo.setPosition(diffyRPos);
                        break;
                    default :
                        break;
                } // switch()
            } // left bumper

            //================ RIGHT BUMPER INCREASES SERVO POSITION ================
            else if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last)
            {
                switch( selectedServo ) { // 1=claw, 2=diffyL, 3=diffyR
                    case 1 :
                        clawPos += stepSize;
                        if( clawPos < 0.0 ) clawPos = 0.0;
                        if( clawPos > 1.0 ) clawPos = 1.0;
                        clawServo.setPosition(clawPos);
                        break;
                    case 2 :
                        diffyLPos += stepSize;
                        if( diffyLPos < 0.0 ) diffyLPos = 0.0;
                        if( diffyLPos > 1.0 ) diffyLPos = 1.0;
                        diffyLServo.setPosition(diffyLPos);
                        break;
                    case 3 :
                        diffyRPos += stepSize;
                        if( diffyRPos < 0.0 ) diffyRPos = 0.0;
                        if( diffyRPos > 1.0 ) diffyRPos = 1.0;
                        diffyRServo.setPosition(diffyRPos);
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

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
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

} // TeleopDiffyWrist
