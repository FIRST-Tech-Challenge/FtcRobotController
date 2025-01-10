/* FTC Team 7572 - Version 1.0 (12/21/2024) */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp Servo Test Program
 */
@TeleOp(name="Teleop-PositionTest", group="Test")
@Disabled
public class TeleopPositionTest extends LinearOpMode {
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
    double  elbowPos, wristPos, clawPos;
    double  stepSize = 0.01;

    double  viperPower = 0.0;
    boolean tiltAngleTweaked = false; // Reminder to zero power when TILT input stops
    boolean liftTweaked      = false; // Reminder to zero power when LIFT input stops

    long    nanoTimeCurr=0, nanoTimePrev=0;
    double  elapsedTime, elapsedHz;

    /* Declare OpMode members. */
//  Hardware2025Bot robot = new Hardware2025Bot(telemetry);
    Hardware2025Bot robot = new Hardware2025Bot();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware (autonomous=true initializes servos)
        robot.init(hardwareMap,true);

        // Preload each variable with the initialization position
        elbowPos = robot.ELBOW_SERVO_INIT;
        wristPos = robot.WRIST_SERVO_INIT;
        clawPos  = robot.CLAW_SERVO_INIT;
        //geckoOn = false;

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
//                  telemetry.addData("SELECTED:", "IntakeServo" );
//                  telemetry.addData("Intake Servo power", "%.1f", robot.geckoServo.getPower() );
                    telemetry.addData("SELECTED:", "ClawServo" );
                    telemetry.addData("Claw Servo Position", "%.1f", robot.clawServo.getPosition());
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
                        clawPos -= stepSize;
                        if( clawPos < 0.0 ) clawPos = 0.0;
                        if( clawPos > 1.0 ) clawPos = 1.0;
                        robot.clawServo.setPosition(clawPos);
/*
                        if(geckoOn){
                            robot.geckoServo.setPower(0.0);
                            geckoOn = false;
                        }
                        else{
                            robot.geckoServo.setPower(-1.0);
                            geckoOn = true;
                        }
*/
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
                        clawPos += stepSize;
                        if( clawPos < 0.0 ) clawPos = 0.0;
                        if( clawPos > 1.0 ) clawPos = 1.0;
                        robot.clawServo.setPosition(clawPos);
/*
                        if(geckoOn){
                            robot.geckoServo.setPower(0.0);
                            geckoOn = false;
                        }
                        else{
                            robot.geckoServo.setPower(1.0);
                            geckoOn = true;
                        }
*/
                        break;
                    default :
                        break;
                } // switch()
            } // right bumper

            processTiltControls();
            ProcessViperLiftControls();

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            elapsedTime  = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;   // msec
            elapsedHz    =  1000.0 / elapsedTime;

            // Update telemetry data
            telemetry.addData("Tilt", "%.1f deg", robot.armTiltAngle);
            telemetry.addData("Viper", "%d counts", robot.viperMotorPos );
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

    /*---------------------------------------------------------------------------------*/
    void processTiltControls() {
        // The encoder is backwards from our definition of MAX and MIN. Maybe change the
        // convention in hardware class?
        boolean safeToManuallyLower = (robot.armTiltAngle > Hardware2025Bot.TILT_ANGLE_HW_MIN_DEG);
        boolean safeToManuallyRaise = (robot.armTiltAngle < Hardware2025Bot.TILT_ANGLE_HW_MAX_DEG);
        double  gamepad1_right_stick = gamepad1.right_stick_y;
        boolean manual_tilt_control = ( Math.abs(gamepad1_right_stick) > 0.08 );

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 CROSS button
        if( gamepad1_cross_now && !gamepad1_cross_last)
        {
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 LEFT BUMPER
        else if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last )
        {
        }

        //===================================================================
        else if( manual_tilt_control || tiltAngleTweaked) {
            // Does user want to rotate turret DOWN (negative joystick input)
            if( safeToManuallyLower && (gamepad1_right_stick < -0.08) ) {
                double motorPower = 0.95 * gamepad1_right_stick; // NEGATIVE
                robot.wormTiltMotor.setPower( motorPower );   // -8% to -95%
                tiltAngleTweaked = true;
            }
            // Does user want to rotate turret UP (positive joystick input)
            else if( safeToManuallyRaise && (gamepad1_right_stick > 0.08) ) {
                double motorPower = 0.95 * gamepad1_right_stick; // POSITIVE
                robot.wormTiltMotor.setPower( motorPower );   // +8% to +95%
                tiltAngleTweaked = true;
            }
            // No more input?  Time to stop turret movement!
            else if(tiltAngleTweaked) {
                robot.wormTiltMotor.setPower( 0.0 );
                tiltAngleTweaked = false;
            }
        } // manual_tilt_control

    } // processTiltControls

    /*---------------------------------------------------------------------------------*/
    void ProcessViperLiftControls() {
        boolean safeToManuallyRetract = (robot.viperMotorPos > Hardware2025Bot.VIPER_EXTEND_ZERO);
        boolean safeToManuallyExtend  = (robot.viperMotorPos < Hardware2025Bot.VIPER_EXTEND_FULL2);
        // Capture user inputs ONCE, in case they change during processing of this code
        // or we want to scale them down
        double  gamepad1_left_trigger  = gamepad1.left_trigger  * 0.5;  // fine control, not speed
        double  gamepad1_right_trigger = gamepad1.right_trigger * 0.5;
        boolean manual_lift_control = ( (gamepad1_left_trigger  > 0.25) || (gamepad1_right_trigger > 0.25) );

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 DPAD UP
        if( gamepad1_dpad_up_now && !gamepad1_dpad_up_last)
        {
        }
        // Check for an OFF-to-ON toggle of the gamepad1 DPAD RIGHT
        else if( gamepad1_dpad_right_now && !gamepad1_dpad_right_last)
        {
        }
        // Check for an OFF-to-ON toggle of the gamepad1 DPAD DOWN
        else if( gamepad1_dpad_down_now && !gamepad1_dpad_down_last)
        {
        }
        // Check for an OFF-to-ON toggle of the gamepad1 DPAD RIGHT
        else if( gamepad1_dpad_left_now && !gamepad1_dpad_left_last)
        {
        }
        //===================================================================
        else if( manual_lift_control || liftTweaked ) {
            // Does user want to manually RAISE the lift?
            if( safeToManuallyExtend && (gamepad1_right_trigger > 0.25) ) {
                viperPower = gamepad1_right_trigger;
                robot.viperMotor.setPower( viperPower );  // fixed power? (robot.VIPER_RAISE_POWER)
                liftTweaked = true;
            }
            // Does user want to manually LOWER the lift?
            else if( safeToManuallyRetract && (gamepad1_left_trigger > 0.25) ) {
                viperPower = robot.VIPER_LOWER_POWER;
                robot.viperMotor.setPower( viperPower );
                liftTweaked = true;
            }
            // No more input?  Time to stop lift movement!
            else if( liftTweaked ) {
                // if the lift is near the bottom, truly go to zero power
                // but if in a raised position, only drop to minimal holding power
                boolean closeToZero = (Math.abs(robot.viperMotorPos - Hardware2025Bot.VIPER_EXTEND_ZERO) < 20);
                viperPower = closeToZero? 0.0 : robot.VIPER_HOLD_POWER;
                robot.viperMotor.setPower( viperPower );
                liftTweaked = false;
            }
        } // manual_lift_control

    }  // ProcessLiftControls


} // TeleopPositionTest
