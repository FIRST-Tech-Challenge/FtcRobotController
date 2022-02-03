/* FTC Team 7572 - Version 2.0 (12/10/2021)
*/
package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * TeleOp Full Control.
 */
@TeleOp(name="Teleop-Skunkworks", group="7592")
@Disabled
public class TestGreatNewFunctionality extends LinearOpMode {
    boolean gamepad1_triangle_last,   gamepad1_triangle_now   = false;  // Capping arm score position
    boolean gamepad1_circle_last,     gamepad1_circle_now     = false;  // Duck motor control
    boolean gamepad1_cross_last,      gamepad1_cross_now      = false;  // Capping arm claw open/close
    boolean gamepad1_square_last,     gamepad1_square_now     = false;  // Capping arm collect/store positions
    boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;  // gamepad1.dpad_up used live/realtime
    boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;  //   (see processDpadDriveMode() below)
    boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;

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
    PIDFCoefficients newPIDF = new PIDFCoefficients( 10.0, 3.0, 0.0, 12.0 );
    PIDFCoefficients oldPIDF;

    // Configure a motor
    public void configureMotor() {
        oldPIDF = robot.cappingMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.cappingMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.cappingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.cappingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
    }


    public void createLog() {
        final String BASE_FOLDER_NAME = "FIRST";

        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        //noinspection ResultOfMethodCallIgnored
        directory.mkdirs();

        try {
            log = new FileWriter(directoryPath+"/"+"autopilotData.txt", false);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    final int CAPPING_CYCLECOUNT_START = 30;  // Capping Arm just started moving (1st cycle)
    final int CAPPING_CYCLECOUNT_SERVO = 20;  // Capping Arm off chassis (safe to rotate wrist servo)
    final int CAPPING_CYCLECOUNT_CHECK = 1;   // Time to check if Capping Arm is still moving?
    final int CAPPING_CYCLECOUNT_DONE  = 0;   // Movement is complete (cycle count is reset)
    boolean clawServoOpen   = false;  // true=OPEN; false=CLOSED on team element
    double    wristServoPos = 0.950;          // Servo setting to target once arm movement starts (WRIST_SERVO_INIT)
    int       cappingArmCycleCount     = CAPPING_CYCLECOUNT_DONE;
    boolean   cappingArmTweaked        = false;  // Reminder to zero power when joystick released

    /*---------------------------------------------------------------------------------*/
    void processCappingArmControls() {
        // Check for an OFF-to-ON toggle of the gamepad1 CROSS button
        if( gamepad1_cross_now && !gamepad1_cross_last)
        {
            if( clawServoOpen ) {
                robot.clawServo.setPosition( robot.CLAW_SERVO_CLOSED );
            }
            else {
                robot.clawServo.setPosition( robot.CLAW_SERVO_OPEN );
            }
            clawServoOpen = !clawServoOpen;
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 TRIANGLE button
        if( gamepad1_triangle_now && !gamepad1_triangle_last)
        {
            // <MIN> STORE ... midpoint1 ... CAP ... midpoint2 ... GRAB <MAX>
            int midpoint1 = (robot.CAPPING_ARM_POS_STORE + robot.CAPPING_ARM_POS_CAP)/2;
            int midpoint2 = (robot.CAPPING_ARM_POS_CAP   + robot.CAPPING_ARM_POS_GRAB)/2;
            // toggle into and out of CAP position (use current arm position to decide)
            if( (robot.cappingMotorPos < midpoint1) ||   /* currently STORE */
                    (robot.cappingMotorPos > midpoint2) )    /* currently GRAB  */
            {  // switch to CAP
                wristServoPos =  robot.WRIST_SERVO_CAP;
                robot.cappingArmPosition( robot.CAPPING_ARM_POS_CAP, 0.70 );
                cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
            }
            else
            { // currently CAP; switch to STORE
                wristServoPos = robot.WRIST_SERVO_STORE;
                robot.cappingArmPosition( robot.CAPPING_ARM_POS_STORE, 0.70 );
                cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
            }
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 SQUARE button
        else if( gamepad1_square_now && !gamepad1_square_last)
        {
            // toggle between GRAB and STORE positions
            // (use current arm position to decide)
            if( robot.cappingMotorPos < robot.CAPPING_ARM_POS_CAP )
            {  // currently STORE; switch to GRAB
                wristServoPos = robot.WRIST_SERVO_GRAB;
                robot.cappingArmPosition( robot.CAPPING_ARM_POS_GRAB, 0.70 );
                cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
            }
            else
            { // currently GRAB; switch to STORE
                wristServoPos = robot.WRIST_SERVO_STORE;
                robot.cappingArmPosition( robot.CAPPING_ARM_POS_STORE, 0.70 );
                cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
            }
        }
        //===================================================================
        if( cappingArmCycleCount > CAPPING_CYCLECOUNT_SERVO ) {
            // nothing to do yet (just started moving)
            cappingArmCycleCount--;
        }
        else if( cappingArmCycleCount == CAPPING_CYCLECOUNT_SERVO ) {
            robot.wristServo.setPosition( wristServoPos );
            cappingArmCycleCount--;
        }
        else if( cappingArmCycleCount > CAPPING_CYCLECOUNT_CHECK ) {
            // nothing to do yet (too soon to check motor state)
            cappingArmCycleCount--;
        }
        else if( cappingArmCycleCount == CAPPING_CYCLECOUNT_CHECK ) {
            if( robot.cappingMotor.isBusy() ) {
                // still moving; hold at this cycle count
            }
            else { // no longer busy; turn off motor power
                robot.cappingMotor.setPower( 0.0 );
                robot.cappingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                cappingArmCycleCount = CAPPING_CYCLECOUNT_DONE;   // ensure we're reset
            }
        }
        else { // cappingArmCycleCount == CAPPING_CYCLECOUNT_DONE
            // arm must be idle; check for manual arm control
            cappingArmCycleCount = CAPPING_CYCLECOUNT_DONE;   // ensure we're reset
            double gamepad1_left_trigger  = gamepad1.left_trigger;
            double gamepad1_right_trigger = gamepad1.right_trigger;
            if( gamepad1_left_trigger > 0.05 ) {
                // limit how far we can drive this direction
                if( robot.cappingMotorPos < robot.CAPPING_ARM_POS_GRAB ) {
                    robot.cappingMotor.setPower( +0.10 * gamepad1_left_trigger );
                    cappingArmTweaked = true;
                }
                else {
                    robot.cappingMotor.setPower( 0.0 );
                }
            }
            else if( gamepad1_right_trigger > 0.05 ) {
                // limit how far we can drive this direction
                if( robot.cappingMotorPos > 0 ) {
                    robot.cappingMotor.setPower( -0.10 * gamepad1_right_trigger );
                    cappingArmTweaked = true;
                }
                else {
                    robot.cappingMotor.setPower( 0.0 );
                }
            }
            else if( cappingArmTweaked ) {
                robot.cappingMotor.setPower( 0.0 );
                cappingArmTweaked = false;
            }
        }
        //===================================================================
        if( gamepad1.left_bumper ) {
            // What was the last commanded position?
            double curPos = robot.wristServo.getPosition();
            if( curPos >  -0.95 ) {
                double newPos = curPos - 0.005;
                robot.wristServo.setPosition( newPos );
            }
        }
        else if( gamepad1.right_bumper ) {
            // What was the last commanded position?
            double curPos = robot.wristServo.getPosition();
            if( curPos <  0.95 ) {
                double newPos = curPos + 0.005;
                robot.wristServo.setPosition( newPos );
            }
        }
    } // processCappingArmControls

    String currentPIDFAdjust = "P";
    public void processPIDFAdjustControls() {
        boolean PIDFUpdated = false;

        if( gamepad1_dpad_up_now && !gamepad1_dpad_up_last)
        {
            PIDFUpdated = true;
            switch(currentPIDFAdjust) {
                case "P":
                    newPIDF.p = newPIDF.p + 1.0;
                    break;
                case "I":
                    newPIDF.i = newPIDF.i + 1.0;
                    break;
                case "D":
                    newPIDF.d = newPIDF.d + 1.0;
                    break;
                case "F":
                    newPIDF.f = newPIDF.f + 1.0;
                    break;
            }
        }
        if( gamepad1_dpad_down_now && !gamepad1_dpad_down_last)
        {
            PIDFUpdated = true;
            switch(currentPIDFAdjust) {
                case "P":
                    if(newPIDF.p > 0.0) {
                        newPIDF.p = newPIDF.p - 1.0;
                    }
                    break;
                case "I":
                    if(newPIDF.i > 0.0) {
                        newPIDF.i = newPIDF.i - 1.0;
                    }
                    break;
                case "D":
                    if(newPIDF.d > 0.0) {
                        newPIDF.d = newPIDF.d - 1.0;
                    }
                    break;
                case "F":
                    if(newPIDF.f > 0.0) {
                        newPIDF.f = newPIDF.f - 1.0;
                    }
                    break;
            }
        }
        if( gamepad1_dpad_right_now && !gamepad1_dpad_right_last)
        {
            switch(currentPIDFAdjust) {
                case "P":
                    currentPIDFAdjust = "I";
                    break;
                case "I":
                    currentPIDFAdjust = "D";
                    break;
                case "D":
                    currentPIDFAdjust = "F";
                    break;
                case "F":
                    currentPIDFAdjust = "P";
                    break;
            }
        }
        if( gamepad1_dpad_left_now && !gamepad1_dpad_left_last)
        {
            switch(currentPIDFAdjust) {
                case "P":
                    currentPIDFAdjust = "F";
                    break;
                case "I":
                    currentPIDFAdjust = "P";
                    break;
                case "D":
                    currentPIDFAdjust = "I";
                    break;
                case "F":
                    currentPIDFAdjust = "D";
                    break;
            }
        }
        if(PIDFUpdated) {
            configureMotor();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap,false);

        // Set the PIDF on the capping motor
        configureMotor();

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

            // Hopefully this does everything capping arm.
            processCappingArmControls();
            processPIDFAdjustControls();
            telemetry.addData("Current PIDF Adjust: ", currentPIDFAdjust);
            telemetry.addData("New PIDF: ", newPIDF.toString());
            telemetry.addData("Old PIDF: ", oldPIDF.toString());
            telemetry.update();
        }
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
