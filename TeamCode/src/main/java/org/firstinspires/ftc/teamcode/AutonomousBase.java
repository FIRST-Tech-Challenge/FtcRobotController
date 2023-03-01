package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwareSlimbot.UltrasonicsInstances.SONIC_RANGE_FRONT;
import static org.firstinspires.ftc.teamcode.HardwareSlimbot.UltrasonicsModes.SONIC_FIRST_PING;
import static org.firstinspires.ftc.teamcode.HardwareSlimbot.UltrasonicsModes.SONIC_MOST_RECENT;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import android.os.Environment;
import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public abstract class AutonomousBase extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareSlimbot robot = new HardwareSlimbot();

    static final int     DRIVE_TO             = 1;       // ACCURACY: tighter tolerances, and slows then stops at final position
    static final int     DRIVE_THRU           = 2;       // SPEED: looser tolerances, and leave motors running (ready for next command)
    static final double  P_DRIVE_COEFF        = 0.005;   // Larger is more responsive, but also less stable
    static final double  HEADING_THRESHOLD    = 2.0;     // Minimum of 1 degree for an integer gyro
    static final double  P_TURN_COEFF         = 0.050;   // Larger is more responsive, but also less stable
    static final double  DRIVE_SPEED_10       = 0.10;    // Lower speed for moving from a standstill
    static final double  DRIVE_SPEED_20       = 0.20;    // Lower speed for moving from a standstill
    static final double  DRIVE_SPEED_30       = 0.30;    // Lower speed for fine control going sideways
    static final double  DRIVE_SPEED_40       = 0.40;    // Normally go slower to achieve better accuracy
    static final double  DRIVE_SPEED_50       = 0.50;    //
    static final double  DRIVE_SPEED_55       = 0.55;    //
    static final double  DRIVE_SPEED_60       = 0.60;    //
    static final double  DRIVE_SPEED_70       = 0.70;    //
    static final double  DRIVE_SPEED_75       = 0.75;    //
    static final double  DRIVE_SPEED_80       = 0.80;    //
    static final double  DRIVE_SPEED_90       = 0.90;    //
    static final double  DRIVE_SPEED_100      = 1.00;    //
    static final double  TURN_SPEED_20        = 0.20;    //
    static final double  TURN_SPEED_30        = 0.30;    //
    static final double  TURN_SPEED_40        = 0.40;    //
    static final double  TURN_SPEED_50        = 0.50;    //
    static final double  TURN_SPEED_55        = 0.55;    //
    static final double  TURN_SPEED_60        = 0.60;    //
    static final double  TURN_SPEED_70        = 0.70;    //
    static final double  TURN_SPEED_80        = 0.80;    //
    static final double  TURN_SPEED_90        = 0.90;    //
    static final double  TURN_SPEED_100       = 1.00;    //
    static final double STRAFE_MULTIPLIER = 1.5;
    static final double MIN_SPIN_RATE      = 0.05;    // Minimum power to turn the robot
    static final double MIN_DRIVE_POW      = 0.05;    // Minimum speed to move the robot
    static final double MIN_DRIVE_MAGNITUDE = Math.sqrt(MIN_DRIVE_POW*MIN_DRIVE_POW+MIN_DRIVE_POW*MIN_DRIVE_POW);

    //Files to access the algorithm constants
    File wheelBaseSeparationFile  = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    double robotEncoderWheelDistance            = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * robot.COUNTS_PER_INCH2;
    double horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    // NOTE: Initializing the odometry global X-Y and ANGLE to 0-0 and 0deg means the frame of reference for all movements is
    // the starting positiong/orientation of the robot.  An alternative is to make the bottom-left corner of the field the 0-0
    // point, with 0deg pointing forward.  That allows all absolute driveToPosition() commands to be an absolute x-y location
    // on the field.
    double robotGlobalXCoordinatePosition       = 0.0;   // in odometer counts
    double robotGlobalYCoordinatePosition       = 0.0;
    double robotOrientationRadians              = 0.0;   // 0deg (straight forward)
    
    double autoXpos                             = 0.0;   // Keeps track of our Autonomous X-Y position and Angle commands. 
    double autoYpos                             = 0.0;   // (useful when a given value remains UNCHANGED from one
    double autoAngle                            = 0.0;   // movement to the next, or INCREMENTAL change from current location).

    double currentPositionX                     = 0.0;   // Keeps track of our Autonomous alignToPole() target position
    double currentPositionY                     = 0.0;
    double currentPositionAngle                 = 0.0;

    double targetPositionX                      = 0.0;   // Keeps track of our Autonomous alignToPole() target position
    double targetPositionY                      = 0.0;
    double targetPositionAngle                  = 0.0;

    double targetDistanceX                      = 0.0;   // How far do we need to move robot base to alignToPole()?
    double targetDistanceY                      = 0.0;

    double targetAngle                          = 0.0;   // Keeps track of our Autonomous alignToPole() target turret angle

    String      storageDir;
    boolean     alignToFront    = true;  // Use front facing camera or back facing
    boolean     blueAlliance    = true;  // Is alliance BLUE (true) or RED (false)?
    boolean     forceAlliance   = false; // Override vision pipeline? (toggled during init phase of autonomous)
    int         fiveStackHeight = 5;     // Number of cones remaining on the 5-stack (always starts at 5)
    int         fiveStackCycles = 3;     // How many we want to attempt to collect/score? (adjustable during init)
    ElapsedTime autonomousTimer = new ElapsedTime();

    double[] timePoleArrive   = {0,0,0,0,0,0};  // Time to drive to pole
    double[] timePoleScore    = {0,0,0,0,0,0};  // Time to alignToPole() and scoreCone()
    double[] timePoleDepart   = {0,0,0,0,0,0};  // Time we depart the pole
    double[] timeStackArrive  = {0,0,0,0,0,0};  // Time we arrive at 5-stack using moveToConeStack()
    double[] timeStackCollect = {0,0,0,0,0,0};  // Time to collect the next cone off the 5-stack
    double[] timeStackDepart  = {0,0,0,0,0,0};  // Time we depart the 5-stack
    double   timeNow;
    int      timeIndex = 0;

    // gamepad controls for changing autonomous options
    boolean gamepad1_circle_last,   gamepad1_circle_now  =false;
    boolean gamepad1_cross_last,    gamepad1_cross_now   =false;
    boolean gamepad1_l_bumper_last, gamepad1_l_bumper_now=false;
    boolean gamepad1_r_bumper_last, gamepad1_r_bumper_now=false;
    boolean gamepad1_dpad_up_last, gamepad1_dpad_up_now = false;
    boolean gamepad1_dpad_down_last, gamepad1_dpad_down_now = false;
    boolean gamepad1_dpad_left_last, gamepad1_dpad_left_now = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;

    // Vision stuff
    PowerPlaySuperPipeline pipelineLow;
    PowerPlaySuperPipeline pipelineFront;
    PowerPlaySuperPipeline pipelineBack;

    int coneNumber = 0;
    boolean scoreHighGoal[] = {true, true, true, true, true, true};

    /*---------------------------------------------------------------------------------*/
    void captureGamepad1Buttons() {
        gamepad1_circle_last   = gamepad1_circle_now;      gamepad1_circle_now   = gamepad1.circle;
        gamepad1_cross_last    = gamepad1_cross_now;       gamepad1_cross_now    = gamepad1.cross;
        gamepad1_l_bumper_last = gamepad1_l_bumper_now;    gamepad1_l_bumper_now = gamepad1.left_bumper;
        gamepad1_r_bumper_last = gamepad1_r_bumper_now;    gamepad1_r_bumper_now = gamepad1.right_bumper;
        gamepad1_dpad_up_last    = gamepad1_dpad_up_now;     gamepad1_dpad_up_now    = gamepad1.dpad_up;
        gamepad1_dpad_down_last  = gamepad1_dpad_down_now;   gamepad1_dpad_down_now  = gamepad1.dpad_down;
        gamepad1_dpad_left_last  = gamepad1_dpad_left_now;   gamepad1_dpad_left_now  = gamepad1.dpad_left;
        gamepad1_dpad_right_last = gamepad1_dpad_right_now;  gamepad1_dpad_right_now = gamepad1.dpad_right;
    } // captureGamepad1Buttons

    /*---------------------------------------------------------------------------------*/
    public void performEveryLoop() {
        robot.readBulkData();
        globalCoordinatePositionUpdate();
        robot.liftPIDPosRun(false);
        robot.turretPIDPosRun(false);
    } // performEveryLoop

    // Create a time stamped folder in
    public void createAutoStorageFolder( boolean isBlue, boolean isLeft ) {
        // Create a subdirectory based on DATE
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());
        String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
        storageDir = Environment.getExternalStorageDirectory().getPath() + "//FIRST//Webcam//" + dateString;

        if (isBlue) {
            if (isLeft) {
                storageDir += "//blue_left//";
            } else {
                storageDir += "//blue_right//";
            }
        } else {
            if (isLeft) {
                storageDir += "//red_left//";
            } else {
                storageDir += "//red_right//";
            }
        }
        storageDir += timeString;

        // Create the directory structure to store the autonomous image used to start auto.
        File baseDir = new File(storageDir);
        baseDir.mkdirs();
    }

    /*---------------------------------------------------------------------------------*/
    // DEPRICATED!  The ultrasonic range sensors return values in whole numbers of centimeters
    // so there's no need to use double-precision variables.
    void distanceFromFront(double desiredDistance, double distanceTolerance) {
        // Value in inches?
        int atRange = 0;
        double range = robot.singleSonarRangeF();
        double rangeErr = range - desiredDistance;
        while (opModeIsActive() && (atRange < 7)) {
            performEveryLoop();
            range = robot.singleSonarRangeF();
            rangeErr = range - desiredDistance;
            telemetry.addData("Sonar Range (F)", "Set: %.1f  Range: %.1f Err: %.1f", desiredDistance, range, rangeErr);
            telemetry.update();
            if(abs(rangeErr) <= distanceTolerance){
                robot.stopMotion();
                atRange++;
            } else {
                atRange = 0;
                robot.driveTrainFwdRev((rangeErr > 0.0) ? +0.10 : -0.10);
            }
        }
        robot.stopMotion();
    } // distanceFromFront

    /*---------------------------------------------------------------------------------*/
    void distanceFromFront( int desiredDistance, int distanceTolerance ) {   // centimeters
        // We're in position, so trigger the first ultrasonic ping
        // NOTE: sonar ranges only update every 50 msec (no matter how much faster we loop
        // than that.  That implies fast control loops will have multiple cycles that use
        // the same range value before a new/different "most recent" ultrasonic range occurs.
        robot.fastSonarRange( HardwareSlimbot.UltrasonicsInstances.SONIC_RANGE_FRONT, HardwareSlimbot.UltrasonicsModes.SONIC_FIRST_PING );
        // Wait 50 msec (DEFAULT_SONAR_PROPAGATION_DELAY_MS)
        sleep( 50 );
        // Begin the control loop.  We use a for() loop to avoid looping forever.  Be
        // sure the desired distances can be achieved in 100 loops, or increase this counter.
        for( int loops=0; loops<100; loops++ ) {
            // Has autonomous ended or been aborted?
            if( opModeIsActive() == false ) break;
            // Handle background tasks
            performEveryLoop();
            // Query the current distance
            int currDistance = robot.fastSonarRange( HardwareSlimbot.UltrasonicsInstances.SONIC_RANGE_FRONT, HardwareSlimbot.UltrasonicsModes.SONIC_MOST_RECENT );
            // Compute the range error
            int rangeErr = currDistance - desiredDistance;
            telemetry.addData("Sonar Range (F)", "Set: %d  Range: %d Err: %d", desiredDistance, currDistance, rangeErr);
            telemetry.update();
            // Are we done?
            if( Math.abs(rangeErr) <= distanceTolerance ) break;
            // Initiate robot movement closer/further to achieve desired range
            robot.driveTrainFwdRev((rangeErr > 0.0) ? +0.10 : -0.10);
            // Don't loop too fast (there's no point) but don't wait too long in case
            // we "just missed" the most recent ultrasonic range measurement update
            sleep( 10 );
        } // for()
        robot.stopMotion();
    } // distanceFromFront

    /*---------------------------------------------------------------------------------*/
    /*  AUTO: Drive at specified angle and power while turning at specified power.     */
    /*---------------------------------------------------------------------------------*/
    void driveAndRotateTurretAngle(double drivePower, double turretPower, boolean turretFacingFront) {
        double frontRight, frontLeft, rearRight, rearLeft, maxPower, xTranslation, yTranslation;
        double turretAngle = robot.turretAngle;

        // Correct the angle for the turret being in the back.
        if(!turretFacingFront) {
            turretAngle = AngleWrapDegrees(turretAngle + 180.0);
        }

        yTranslation = drivePower * Math.cos(toRadians(turretAngle));
        xTranslation = drivePower * Math.sin(toRadians(turretAngle));

        frontLeft  = yTranslation + xTranslation;
        frontRight = yTranslation - xTranslation;
        rearLeft   = yTranslation - xTranslation;
        rearRight  = yTranslation + xTranslation;

        // Normalize the values so none exceed +/- 1.0
        maxPower = Math.max( Math.max( Math.abs(rearLeft),  Math.abs(rearRight)  ),
                Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        if (maxPower > 1.0)
        {
            rearLeft   /= maxPower;
            rearRight  /= maxPower;
            frontLeft  /= maxPower;
            frontRight /= maxPower;
        }
        // Update motor power settings:
        robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight );
        robot.turretMotorSetPower(turretPower);
    } // driveAndRotateTurretAngle

    void alignToPole(boolean turretFacingFront, boolean fromStack, boolean highJunction ) {
        PowerPlaySuperPipeline alignmentPipeline;
        PowerPlaySuperPipeline.AnalyzedPole theLocalPole;
        final double DRIVE_SLOPE  = 0.004187;
        final double DRIVE_OFFSET = 0.04522;
        final int CYCLES_AT_ANGLE    = 4;
        final int CYCLES_AT_DISTANCE = 3;

        PIDControllerTurret pidController = new PIDControllerTurret(0.00008,0.0, 0.0, 0.070 );

        double turretPower;
        double turretPowerMax = 0.10;  // maximum we don't want the PID to exceed
        double drivePower;

        double startTime = autonomousTimer.milliseconds();
        double abortTime = startTime + 3000.0;  // abort after 3 seconds

        // If we add back front camera, use boolean to determine which pipeline to use.
        alignmentPipeline = turretFacingFront ? pipelineFront : pipelineBack;
        theLocalPole = alignmentPipeline.getDetectedPole();

		// Save the starting image we have to correct for
        alignmentPipeline.savePoleAutoImage();
        int properDistanceCount;
        boolean properDistance;
        double offset;

        if( highJunction ) {
            if( fromStack ) {
                properDistanceCount = theLocalPole.properDistanceHighStackCount;
                properDistance = theLocalPole.properDistanceStackHigh;
                offset = theLocalPole.highDistanceStackOffset;
            } else {
                properDistanceCount = theLocalPole.properDistanceHighCount;
                properDistance = theLocalPole.properDistanceHigh;
                offset = theLocalPole.highDistanceOffset;
            }
        } else {
            if( fromStack ) {
                properDistanceCount = theLocalPole.properDistanceMedStackCount;
                properDistance = theLocalPole.properDistanceStackMed;
                offset = theLocalPole.medDistanceStackOffset;
            } else {
                properDistanceCount = theLocalPole.properDistanceMedCount;
                properDistance = theLocalPole.properDistanceMed;
                offset = theLocalPole.medDistanceOffset;
            }
        }

        while (opModeIsActive() && ((theLocalPole.alignedCount <= CYCLES_AT_ANGLE) ||
                properDistanceCount <= CYCLES_AT_DISTANCE)) {
            performEveryLoop();
            turretPower = pidController.update(0.0, theLocalPole.centralOffset);
            // Ensure we never exceed a safe power
            if( turretPower > +turretPowerMax ) turretPower = +turretPowerMax;
            if( turretPower < -turretPowerMax ) turretPower = -turretPowerMax;

            if(properDistance) {
                drivePower = 0.0;
            } else {
                // Need to calculate the drive power based on pixel offset
                // Maximum number of pixels off would be in the order of 30ish.
                // This is a first guess that will have to be experimented on.
                // Go 1.0 to 0.08 from 30 pixels to 2.
                drivePower = (offset > 0 )?
                        (offset * DRIVE_SLOPE + DRIVE_OFFSET) :
                        (offset * DRIVE_SLOPE - DRIVE_OFFSET);
            }

            telemetry.addData("alignToPole", "ang=%d (%.1f) dist=%d (%.1f)",
                    theLocalPole.alignedCount, theLocalPole.centralOffset,
                    properDistanceCount, offset );
            telemetry.update();

            if(abs(drivePower) < 0.01 && abs(turretPower) < 0.01) {
                robot.stopMotion();
                robot.turretMotorSetPower(0);
            } else {
                driveAndRotateTurretAngle(drivePower, turretPower, turretFacingFront);
            }

            // Do we need to abort due to timeout?
            if( autonomousTimer.milliseconds() >= abortTime )
                break;

            // update the image for the next loop
            theLocalPole = alignmentPipeline.getDetectedPole();
            if( highJunction ) {
                if( fromStack ) {
                    properDistanceCount = theLocalPole.properDistanceHighStackCount;
                    properDistance = theLocalPole.properDistanceStackHigh;
                    offset = theLocalPole.highDistanceStackOffset;
                } else {
                    properDistanceCount = theLocalPole.properDistanceHighCount;
                    properDistance = theLocalPole.properDistanceHigh;
                    offset = theLocalPole.highDistanceOffset;
                }
            } else {
                if( fromStack ) {
                    properDistanceCount = theLocalPole.properDistanceMedStackCount;
                    properDistance = theLocalPole.properDistanceStackMed;
                    offset = theLocalPole.medDistanceStackOffset;
                } else {
                    properDistanceCount = theLocalPole.properDistanceMedCount;
                    properDistance = theLocalPole.properDistanceMed;
                    offset = theLocalPole.medDistanceOffset;
                }
            }
        }
        robot.stopMotion();
        robot.turretMotorSetPower(0.0);
        // Save the final image after our adjustments
        alignmentPipeline.savePoleAutoImage();
    } // alignToPole

    /*
     * @param blueCone - true = blue cone stack, false = red cone stack.
     * @param targetDistance - distance in cm from the cone stack to line up.
     */
    void alignToConeStack(boolean blueCone, int targetDistance) {
        PowerPlaySuperPipeline.AnalyzedCone theLocalCone;
        final double DRIVE_SLOPE  = 0.004187;
        final double DRIVE_OFFSET = 0.04522;
        final double TURN_SLOPE   = 0.004187;
        final double TURN_OFFSET  = 0.04522;
        double drivePower;
        double turnPower;
        int coneDistance;
        int distanceError;
        int properDistanceCount = 0;
        // This is in CM
        final int MAX_DISTANCE_ERROR = 1;

        theLocalCone = blueCone ? pipelineLow.getDetectedBlueCone() : pipelineLow.getDetectedRedCone();
        // This first reading just triggers the ultrasonic to send out its first ping, need to wait
        // 50 msec for it to get a result.
        robot.fastSonarRange(SONIC_RANGE_FRONT, SONIC_FIRST_PING);
        sleep(50);

        double startTime = autonomousTimer.milliseconds();
        double abortTime = startTime + 7000.0;  // abort after 7.0 seconds

        while (opModeIsActive() && ((theLocalCone.alignedCount <= 3) ||
                properDistanceCount <= 3)) {
            theLocalCone = blueCone ? pipelineLow.getDetectedBlueCone() : pipelineLow.getDetectedRedCone();
            coneDistance = robot.fastSonarRange(SONIC_RANGE_FRONT, SONIC_MOST_RECENT);
            distanceError = targetDistance - coneDistance;
            if(abs(distanceError) <= MAX_DISTANCE_ERROR) {
                properDistanceCount++;
            } else {
                properDistanceCount = 0;
            }
            drivePower = (distanceError > 0) ? (-distanceError * DRIVE_SLOPE - DRIVE_OFFSET) :
                                               (-distanceError * DRIVE_SLOPE + DRIVE_OFFSET);
            turnPower = (theLocalCone.centralOffset > 0) ?
                    (theLocalCone.centralOffset * TURN_SLOPE + TURN_OFFSET) :
                    (theLocalCone.centralOffset * TURN_SLOPE - TURN_OFFSET);
            driveAndRotate(drivePower, turnPower);
            telemetry.addData("Cone Data", "drvPwr %.2f turnPwr %.2f", drivePower, turnPower);
            telemetry.addData("Cone Data", "coneDst %d dstErr %d offset %.2f", coneDistance,
                    distanceError, theLocalCone.centralOffset);
            telemetry.update();
            // Do we need to abort?
            if( autonomousTimer.milliseconds() >= abortTime )
                break;
        }
        robot.stopMotion();
    }

    void driveAndRotate(double drivePower, double turnPower) {
        double frontRight, frontLeft, rearRight, rearLeft, maxPower;

        frontLeft  = drivePower - turnPower;
        frontRight = drivePower + turnPower;
        rearLeft   = drivePower - turnPower;
        rearRight  = drivePower + turnPower;

        // Normalize the values so none exceed +/- 1.0
        maxPower = Math.max( Math.max( Math.abs(rearLeft),  Math.abs(rearRight)  ),
                Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        if (maxPower > 1.0)
        {
            rearLeft   /= maxPower;
            rearRight  /= maxPower;
            frontLeft  /= maxPower;
            frontRight /= maxPower;
        }
        // Update motor power settings:
        robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight );
    } // driveAndRotate
    /*---------------------------------------------------------------------------------*/
    void rotateToCenterPole() {
        PowerPlaySuperPipeline.AnalyzedPole theLocalPole;
        theLocalPole = pipelineLow.getDetectedPole();
        while (opModeIsActive() && (theLocalPole.alignedCount < 2)) {
            performEveryLoop();
            if(theLocalPole.aligned) {
                robot.stopMotion();
            } else {
                robot.driveTrainTurn((theLocalPole.centralOffset > 0) ? +0.085 : -0.085);
            }
            theLocalPole = pipelineLow.getDetectedPole();
        }
        robot.stopMotion();
        // TODO: can we use this aligned position along the tape to update our known
        // odometry world position? (offsetting any drift-error that has accumulated?
    } // rotateToCenterPole

    /*---------------------------------------------------------------------------------------------
     * getAngle queries the current gyro angle
     * @return  current gyro angle (-179.9 to +180.0)
     */
    protected double getAngle() {
        // calculate error in -179.99 to +180.00 range  (
        double gryoAngle = robot.headingIMU();
        while (gryoAngle >   180.0) gryoAngle -= 360.0;
        while (gryoAngle <= -180.0) gryoAngle += 360.0;
        return gryoAngle;
    } // getAngle()

    /*---------------------------------------------------------------------------------------------
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          Positive error means the robot should turn LEFT (CCW) to reduce error.
     */
    protected double getError(double targetAngle) {
        // calculate error in -179 to +180 range  (
        double robotError = targetAngle - robot.headingIMU();
        while (robotError >  180.0)  robotError -= 360.0;
        while (robotError <= -180.0) robotError += 360.0;
        return robotError;
    } // getError()

    /*---------------------------------------------------------------------------------------------
     * getAngleError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          Positive error means the robot should turn LEFT (CCW) to reduce error.
     */
    protected double getAngleError(double targetAngle) {
        // calculate error in -179 to +180 range  (
        double robotError = targetAngle - robot.headingAngle;
        while (robotError >  180.0)  robotError -= 360.0;
        while (robotError <= -180.0) robotError += 360.0;
        return robotError;
    } // getAngleError()

    /*---------------------------------------------------------------------------------------------
     * returns desired steering force.  +/- 1 range.  positive = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return clippedSteering
     */
    protected double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    } // getSteer()

    /*---------------------------------------------------------------------------------------------
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn( double speed, double angle ) {

        // keep looping while we are still active, and not on heading.
        while( opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, 0.0,0.0) ) {
            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
            performEveryLoop();

            // update range sensor data
//          robot.updateRangeL();
//          robot.updateRangeR();
        }
    } // gyroTurn()

    /*---------------------------------------------------------------------------------------------
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // range check the provided angle
        if( (angle > 360.0) || (angle < -360.0) ) {
            angle = getAngle();  // maintain current angle
        }

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
            performEveryLoop();
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF, 0.0,0.0);
//          telemetry.update();
        }

        // Stop all motion;
        robot.driveTrainMotorsZero();
    } // gyroHold

    /*---------------------------------------------------------------------------------------------
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @param left_right Movement left/right  (positive=left,    negative=right)
     * @param fore_aft   Movement forward/aft (positive=forward, negative=aft)
     * @return onHeading
     */
    boolean onHeading(double speed, double angle, double PCoeff, double left_right, double fore_aft ) {
        double  error;
        double  steer;
        double  baseSpeed;
        double  faSpeed;
        double  lrSpeed;
        boolean onTarget = false;
        double  frontLeft;
        double  frontRight;
        double  backLeft;
        double  backRight;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            frontLeft  = 0.0;  // desired angle achieved; shut down all 4 motors
            frontRight = 0.0;
            backLeft   = 0.0;
            backRight  = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            baseSpeed  = speed * steer;
            lrSpeed    = speed * left_right;  // scale left/right speed (-1.0 .. +1.0) using turn speed
            faSpeed    = speed * fore_aft;    // scale fore/aft   speed (-1.0 .. +1.0) using turn speed
            frontLeft  =  baseSpeed + lrSpeed;  // increases LEFT speed
            frontRight = -baseSpeed + lrSpeed;  // decreased RIGHT speed (less negative)
            backLeft   = frontLeft  - faSpeed;  // decreases
            backRight  = frontRight + faSpeed;  // increases
        }

        // Send desired speeds to motors.
        robot.frontLeftMotor.setPower(frontLeft);
        robot.frontRightMotor.setPower(frontRight);
        robot.rearLeftMotor.setPower(backLeft);
        robot.rearRightMotor.setPower(backRight);

        // Display drive status for the driver.
        if( false ) {
            // Display it for the driver.
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed Front", "%5.2f:%5.2f", frontLeft, frontRight);
            telemetry.addData("Speed Back ", "%5.2f:%5.2f", backLeft, backRight);
            telemetry.update();
        }
        return onTarget;
    } // onHeading()

    /**
     *
     * @param fl Front left motor power
     * @param fr Front right motor power
     * @param bl Back left motor power
     * @param br Back right motor power
     * @return Scaling factor to make sure power level is set to less than 1
     */
    public double scalePower(double fl, double fr, double bl, double br) {
        double scaleFactor = 1.0;
        double maxValue = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br))));
        if(maxValue > 1.0) {
            scaleFactor = 1.0 / maxValue;
        }
        return scaleFactor;
    }

    /**
     * @param leftWall - true strafe to left wall, false strafe to right wall
     * @param maxSpeed - The speed to use when going large distances
     * @param distanceFromWall - The distance to make the robot parallel to the wall in cm
     * @param timeout - The maximum amount of time to wait until giving up
     * @return true if reached distance, false if timeout occurred first
     */
    public boolean strafeToWall(boolean leftWall, double maxSpeed, int distanceFromWall, int timeout) {
        double maxPower = Math.abs(maxSpeed);
        boolean reachedDestination = false;
        int allowedError = 2; // in cm
        double angleErrorMult = 0.014;
        double distanceErrorMult = 0.014;
        ElapsedTime timer = new ElapsedTime();
        int sensorDistance;
        int distanceError;
        performEveryLoop();
        double driveAngle = robot.headingIMU();
        double angleError;
        double rotatePower;
        double drivePower;
        double fl, fr, bl, br;
        double scaleFactor;
        timer.reset();
        while(opModeIsActive() && !reachedDestination && (timer.milliseconds() < timeout)) {
            performEveryLoop();
            //sensorDistance = leftWall ? robot.singleSonarRangeL() : robot.singleSonarRangeR();
            sensorDistance = 10;
            distanceError = sensorDistance - distanceFromWall;
            if(Math.abs(distanceError) > allowedError) {
                // Right is negative angle, left positive
                angleError = getError(driveAngle);
                rotatePower = angleError * angleErrorMult;
                drivePower = distanceError * distanceErrorMult;
                drivePower = leftWall ? drivePower : -drivePower;
                drivePower = Math.copySign(Math.min(Math.max(Math.abs(drivePower), robot.MIN_STRAFE_POW), maxPower), drivePower);
                fl = -drivePower + rotatePower;
                fr = drivePower - rotatePower;
                bl = drivePower + rotatePower;
                br = -drivePower - rotatePower;
                scaleFactor = scalePower(fl, fr, bl, br);
                robot.driveTrainMotors(scaleFactor*fl,
                        scaleFactor*fr,
                        scaleFactor*bl,
                        scaleFactor*br);
            } else {
                robot.driveTrainMotorsZero();
                reachedDestination = true;
            }
        }
        // Timed out
        if(!reachedDestination) {
            robot.driveTrainMotorsZero();
        }

        return reachedDestination;
    } // strafeToWall

    /**
     * @param frontWall - true drive to front wall, false drive to back wall
     * @param maxSpeed - The speed to use when going large distances
     * @param distanceFromWall - The distance to make the robot parallel to the wall in cm
     * @param timeout - The maximum amount of time to wait until giving up
     * @return true if reached distance, false if timeout occurred first
     */
    public boolean driveToWall(boolean frontWall, double maxSpeed, int distanceFromWall, int timeout) {
        double maxPower = Math.abs(maxSpeed);
        boolean reachedDestination = false;
        int allowedError = 2; // in cm
        double angleErrorMult = 0.014;
        double distanceErrorMult = 0.014;
        ElapsedTime timer = new ElapsedTime();
        int sensorDistance;
        int distanceError;
        performEveryLoop();
        double driveAngle = robot.headingIMU();
        double angleError;
        double rotatePower;
        double drivePower;
        double fl, fr, bl, br;
        double scaleFactor;
        timer.reset();
        while(opModeIsActive() && !reachedDestination && (timer.milliseconds() < timeout)) {
            performEveryLoop();
 //           sensorDistance = frontWall ? robot.singleSonarRangeF() : robot.singleSonarRangeB();
            sensorDistance = 10;

            distanceError = sensorDistance - distanceFromWall;
            if(Math.abs(distanceError) > allowedError) {
                angleError = getError(driveAngle);
                rotatePower = angleError * angleErrorMult;
                drivePower = distanceError * distanceErrorMult;
                drivePower = frontWall ? drivePower : -drivePower;
                drivePower = Math.copySign(Math.min(Math.max(Math.abs(drivePower), robot.MIN_DRIVE_POW), maxPower), drivePower);
                fl = drivePower + rotatePower;
                fr = drivePower - rotatePower;
                bl = drivePower + rotatePower;
                br = drivePower - rotatePower;
                scaleFactor = scalePower(fl, fr, bl, br);
                robot.driveTrainMotors(scaleFactor*fl,
                        scaleFactor*fr,
                        scaleFactor*bl,
                        scaleFactor*br);
            } else {
                robot.driveTrainMotorsZero();
                reachedDestination = true;
            }
        }
        // Timed out
        if(!reachedDestination) {
            robot.driveTrainMotorsZero();
        }

        return reachedDestination;
    } // driveToWall

    //============================ TIME-BASED NAVIGATION FUNCTIONS ============================

    /*---------------------------------------------------------------------------------------------
     * Method will drive straight for a specified time.
     * @param speed  Speed to set all motors, positive forward, negative backward
     * @param time   How long to drive (milliseconds)
     */
    public void timeDriveStraight( double speed, int time ) {
        if(opModeIsActive()) {
            robot.driveTrainMotors(speed, speed, speed, speed);
            sleep(time);
            robot.stopMotion();
        }
    }

    /*---------------------------------------------------------------------------------------------
     * Method will strafe straight for a specified time.
     * @param speed  Speed to set all motors, postive strafe left, negative strafe right
     * @param time   How long to strafe (milliseconds)
     */
    public void timeDriveStrafe( double speed, int time ) {
        if(opModeIsActive()) {
            robot.driveTrainMotors(-speed, speed, speed, -speed);
            sleep(time);
            robot.stopMotion();
        }
    }

    //============================ IMU/GYRO-BASED NAVIGATION FUNCTIONS ============================

    /*---------------------------------------------------------------------------------------------
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive( double maxSpeed, boolean driveY, double distance, double angle, int driveType ) {
        double  error, steer;
        double  curSpeed, leftSpeed, rightSpeed, max;
        boolean reachedTarget;
        int     loopCount = 0;
        int     posTol = (driveType == DRIVE_TO)? 15:40;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
            performEveryLoop();

            // range check the provided angle
            if( (angle > 360.0) || (angle < -360.0) ) {
                angle = getAngle();  // maintain current angle
            }

            // configure RUN_TO_POSITION drivetrain motor setpoints
            robot.setRunToPosition( driveY, distance );

            // start motion.
            maxSpeed = Range.clip(Math.abs(maxSpeed), -1.0, 1.0);
            robot.frontLeftMotor.setPower(maxSpeed);
            robot.frontRightMotor.setPower(maxSpeed);
            robot.rearLeftMotor.setPower(maxSpeed);
            robot.rearRightMotor.setPower(maxSpeed);

            // Allow movement to start before checking isBusy()
            sleep( 150 ); // 150 msec

            // keep looping while we are still active, and ALL motors are running.
            while( opModeIsActive() ) {

                // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
                performEveryLoop();

                int frontLeftError  = Math.abs(robot.frontLeftMotorTgt - robot.frontLeftMotorPos);
                int frontRightError = Math.abs(robot.frontRightMotorTgt - robot.frontRightMotorPos);
                int rearLeftError   = Math.abs(robot.rearLeftMotorTgt - robot.rearLeftMotorPos);
                int rearRightError  = Math.abs(robot.rearRightMotorTgt - robot.rearRightMotorPos);
                int avgError = (frontLeftError + frontRightError + rearLeftError + rearRightError) / 4;

                // 19.2:1 is 537 counts/rotation (18.85" distance).  1/2" tolerance = 14.24 counts
                reachedTarget = ((frontLeftError < posTol) && (frontRightError < posTol) &&
                                 (rearLeftError  < posTol) && (rearRightError  < posTol) );
//              reachedTarget = !robot.frontLeftMotor.isBusy() && !robot.frontRightMotor.isBusy() &&
//                              !robot.rearLeftMotor.isBusy() && !robot.rearRightMotor.isBusy();
                if (reachedTarget) break;

                // update range sensor data
//              robot.updateRangeL();
//              robot.updateRangeR();

                // reduce motor speed as we get close so we don't overshoot
                if (avgError < 75) {
                    curSpeed = 0.15;
                } else if (avgError < 300) {
                    curSpeed = 0.75 * maxSpeed;
                }
                else {
                    curSpeed = maxSpeed;
                }

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed  = curSpeed - steer;
                rightSpeed = curSpeed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.frontLeftMotor.setPower(leftSpeed);
                robot.frontRightMotor.setPower(rightSpeed);
                robot.rearLeftMotor.setPower(leftSpeed);
                robot.rearRightMotor.setPower(rightSpeed);

                loopCount++; // still working, or stale data?

                // Display drive status for the driver.
                if( false ) {
                    telemetry.addData("loopCount", "%d", loopCount );
                    telemetry.addData("Err/St", "%5.1f/%5.3f", error, steer);
                    telemetry.addData("Target F", "%7d:%7d", robot.frontLeftMotorTgt, robot.frontRightMotorTgt);
                    telemetry.addData("Target R", "%7d:%7d", robot.rearLeftMotorTgt, robot.rearRightMotorTgt);
                    telemetry.addData("Error F", "%7d:%7d", frontLeftError, frontRightError);
                    telemetry.addData("Error R", "%7d:%7d", rearLeftError, rearRightError);
                    telemetry.addData("Speed", "%5.3f:%5.3f", leftSpeed, rightSpeed);
                    telemetry.update();
                }
            } // opModeIsActive

            if( driveType == DRIVE_TO ) {
                // Stop all motion
                robot.driveTrainMotorsZero();
            }

            // Turn off RUN_TO_POSITION
//          sleep( 100 ); // 100 msec delay before changing modes
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } // opModeIsActive()
    } // gyroDrive()

    //============================ ODOMETRY-BASED NAVIGATION FUNCTIONS ============================
    public void driveToPosition(double yTarget, double xTarget, double angleTarget,
                                   double speedMax, double turnMax, int driveType) {
        // Loop until we get to destination.
        performEveryLoop();
        while(!driveToXY(yTarget, xTarget, angleTarget,
                speedMax, driveType)
                && opModeIsActive()) {
            performEveryLoop();
        }

        // Fix the angle if we didn't reach angle in the drive
        if(driveType == DRIVE_TO) {
            rotateToAngle(angleTarget, turnMax);
        }
    }

    /**
     * @param angleTarget  - The angle the robot should try to face when reaching destination.
     * @param turnMax - Highest value to use for turn speed.
     */
    public void rotateToAngle(double angleTarget,
                                 double turnMax) {
        // Move the robot away from the wall.
        performEveryLoop();
        rotateToAngle(angleTarget, true);
        // Loop until we get to destination.
        performEveryLoop();
        while(!rotateToAngle(angleTarget, false) && opModeIsActive()) {
            performEveryLoop();
        }
    }

    /**
     * @param angleTarget  - The angle the robot should try to face when reaching destination.
     * @param resetDriveAngle - When we start a new drive, need to reset the starting drive angle.
     * @return - Boolean true we have reached destination, false we have not
     */
    protected double lastDriveAngle;
    protected boolean rotateToAngle(double angleTarget, boolean resetDriveAngle) {
        boolean reachedDestination = false;
        double xMovement, yMovement, turnMovement;
        double errorMultiplier = 0.016;
        double turnMin = MIN_SPIN_RATE;
        double deltaAngle = AngleWrapRadians(toRadians(angleTarget) - robotOrientationRadians);
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;

        // This should be set on the first call to start us on a new path.
        if(resetDriveAngle) {
            lastDriveAngle = deltaAngle;
        }

        // We are done if we are within 2 degrees
        if(Math.abs(Math.toDegrees(deltaAngle)) < 2) {
            // We have reached our destination if the angle is close enough
            robot.stopMotion();
            reachedDestination = true;
            // We are done when we flip signs.
        } else if(lastDriveAngle < 0) {
            // We have reached our destination if the delta angle sign flips from last reading
            if(deltaAngle >= 0) {
                robot.stopMotion();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                xMovement = 0;
                yMovement = 0;
                if(turnSpeed > -turnMin) {
                    turnSpeed = -turnMin;
                }
                turnMovement = turnSpeed;
                ApplyMovement(yMovement, xMovement, turnMovement);
            }
        } else {
            // We have reached our destination if the delta angle sign flips
            if(deltaAngle <= 0) {
                robot.stopMotion();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                xMovement = 0;
                yMovement = 0;
                if(turnSpeed < turnMin) {
                    turnSpeed = turnMin;
                }
                turnMovement = turnSpeed;
                ApplyMovement(yMovement, xMovement, turnMovement);
            }
        }
        lastDriveAngle = deltaAngle;

        return reachedDestination;
    }

    /**
     * @param xTarget           - The X field coordinate to go to.
     * @param yTarget           - The Y field coordinate to go to.
     * @param angleTarget - The angle the robot should try to face when reaching destination in degrees.
     * @param speedMin    - The minimum speed that allows movement.
     * @param speedMax    - Sets the maximum speed to drive.
     * @param errorMultiplier - Sets the proportional speed to slow down.
     * @param errorAllowed - Sets the allowable error to claim target reached.
     * @param driveType - Allows waypoint to be a drive through where the robot won't slow down.
     * @return - Boolean true we have reached destination, false we have not
     */
    protected boolean driveToXY(double yTarget, double xTarget, double angleTarget, double speedMin,
                             double speedMax, double errorMultiplier, double errorAllowed,
                             int driveType) {
        boolean reachedDestination = false;
        // Not sure why, but the x and y are backwards
        double xWorld = robotGlobalYCoordinatePosition / robot.COUNTS_PER_INCH2;  // inches (backward! see notes)
        double yWorld = robotGlobalXCoordinatePosition / robot.COUNTS_PER_INCH2;  // inches
        double xMovement, yMovement, turnMovement;
        // Not sure why, but the x and y are backwards
        double deltaX = yTarget - xWorld;
        double deltaY = xTarget - yWorld;
        double driveAngle = Math.atan2(deltaY, deltaX);
        double deltaAngle = AngleWrapRadians(toRadians(angleTarget) - robotOrientationRadians);
        double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double driveSpeed;
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;
        // Have to convert from world angles to robot centric angles.
        double robotDriveAngle = driveAngle - robotOrientationRadians;

        // This will allow us to do multi-point routes without huge slowdowns.
        // Such use cases will be changing angles, or triggering activities at
        // certain points.
        if(!(driveType == DRIVE_THRU)) {
            driveSpeed = magnitude * errorMultiplier;
        } else {
            driveSpeed = speedMax;
        }

        if(driveSpeed < speedMin) {
            driveSpeed = speedMin;
        } else if (driveSpeed > speedMax) {
            driveSpeed = speedMax;
        }

        // Check if we passed through our point
        if(magnitude <= errorAllowed) {
            reachedDestination = true;
            if(!(driveType == DRIVE_THRU)) {
                robot.stopMotion();
            } else {
                // This can happen if the robot is already at error distance for drive through
                xMovement = driveSpeed * Math.cos(robotDriveAngle);
                yMovement = driveSpeed * Math.sin(robotDriveAngle);
                turnMovement = turnSpeed;
                ApplyMovement(yMovement, xMovement, turnMovement);
            }
        } else {
            xMovement = driveSpeed * Math.cos(robotDriveAngle);
            yMovement = driveSpeed * Math.sin(robotDriveAngle);
            turnMovement = turnSpeed;
            ApplyMovement(yMovement, xMovement, turnMovement);
        }

        return reachedDestination;
    }
    /**
     * @param yTarget           - The Y field coordinate to go to.
     * @param xTarget           - The X field coordinate to go to.
     * @param angleTarget   - The angle the robot should try to face when reaching destination in degrees.
     * @param speedMax    - Sets the speed when we are driving through the point.
     * @param driveType     - Slows the robot down to stop at destination coordinate.
     * @return - Boolean true we have reached destination, false we have not
     */
    protected boolean driveToXY(double yTarget, double xTarget, double angleTarget,
                             double speedMax, int driveType) {

        // Convert from cm to inches
        double errorMultiplier = 0.033;
        double speedMin = MIN_DRIVE_MAGNITUDE;
        double allowedError = (driveType == DRIVE_THRU) ? 2.75 : 0.5;

        return (driveToXY(yTarget, xTarget, angleTarget, speedMin, speedMax, errorMultiplier,
                allowedError, driveType));
    }

    // Odometry updates
    private long lastUpdateTime = 0;

    /**converts xMovement, movement_x, movement_turn into motor powers */
    public void ApplyMovement(double yMovement, double xMovement, double turnMovement) {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        // 2.1 is the ratio between the minimum power to strafe, 0.19, and driving, 0.09.
//        double frontLeft = movement_y-movement_turn+movement_x*1.5;
//        double backLeft = movement_y-movement_turn-movement_x*1.5;
//        double backRight = movement_y+movement_turn+movement_x*1.5;
//        double frontRight = movement_y+movement_turn-movement_x*1.5;
        double frontRight = xMovement - yMovement - turnMovement;
        double frontLeft  = xMovement + yMovement + turnMovement;
        double backRight  = xMovement + yMovement - turnMovement;
        double backLeft   = xMovement - yMovement + turnMovement;

        //find the maximum of the powers
        double maxRawPower = Math.abs(frontLeft);
        if(Math.abs(backLeft) > maxRawPower){ maxRawPower = Math.abs(backLeft);}
        if(Math.abs(backRight) > maxRawPower){ maxRawPower = Math.abs(backRight);}
        if(Math.abs(frontRight) > maxRawPower){ maxRawPower = Math.abs(frontRight);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        frontLeft *= scaleDownAmount;
        backLeft *= scaleDownAmount;
        backRight *= scaleDownAmount;
        frontRight *= scaleDownAmount;

        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        robot.driveTrainMotors(frontLeft, frontRight, backLeft, backRight);
    }

    /*--------------------------------------------------------------------------------------------*/
    /**
     * Move robot to specified target position/orientation
     * @param yTarget (inches)
     * @param xTarget (inches)
     * @param angleTarget (degrees; 0deg is straight ahead)
     * @param speedMax
     * @param turnMax
     * @param driveType - DRIVE_TO goes for accuracy and stops all motors; DRIVE_THRU does not
     */
/*
    public void driveToPosition( double yTarget, double xTarget, double angleTarget,
                                  double speedMax, double turnMax, int driveType ) {
        // Loop until we reach the target (or autonomous program aborts)
        while( opModeIsActive() ) {
            // Tend to any automatic mechanism movement updates
            performEveryLoop();
            // Power drivetrain motors to move to where we WANT to be
            if( moveToPosition( yTarget, xTarget, angleTarget, speedMax, turnMax, driveType ) ) {
                // If this is the final destination, then stop all motors
                if( driveType == DRIVE_TO ) {
                    robot.driveTrainMotorsZero();
//                  sleep(30000);   // for debugging individual movements
                }
                // Break out of the loop
                break;
            }
        } // opModeIsActive()
    } // driveToPosition
*/

    /*--------------------------------------------------------------------------------------------*/
    /**
     * Compute instantaneous motor powers needed to move toward the specified target position/orientation
     * NOTE that this system uses X and Y directions OPPOSITE of globalCoordinatePositionUpdate so
     * that we can have straight-forward be 0deg (-90deg left to +90deg right) instead of 0deg left.
     * @param xTarget (inches) <-- pass in desired Y target!
     * @param yTarget (inches) <-- pass in desired X target
     * @param angleTarget (degrees)
     * @param speedMax
     * @param turnMax
     * @param driveType (DRIVE_TO or DRIVE_THRU)
     * @return boolean true/false for DONE?
     */
    public boolean moveToPosition( double xTarget, double yTarget, double angleTarget,
                                    double speedMax, double turnMax, int driveType ) {
        // Convert current robot X,Y position from encoder-counts to inches
        double x_world = robotGlobalYCoordinatePosition / robot.COUNTS_PER_INCH2;  // inches (X/Y backward! see notes)
        double y_world = robotGlobalXCoordinatePosition / robot.COUNTS_PER_INCH2;  // inches
        double angle_world = robotOrientationRadians;                              // radians
        // Compute distance and angle-offset to the target point
        double distanceToPoint   = Math.sqrt( Math.pow((xTarget - x_world),2.0) + Math.pow((yTarget - y_world),2.0) );
        double distToPointAbs    = Math.abs( distanceToPoint );
        double angleToPoint      = (distToPointAbs < 0.001)? angle_world : Math.atan2( (yTarget - y_world), (xTarget - x_world) ); // radians
        double deltaAngleToPoint = AngleWrapRadians( angleToPoint - angle_world );       // radians
        // What tolerance do we use to declare that we're at the target? (inches)
        double xy_tolerance = (driveType == DRIVE_TO)? 0.25 : (3.5 * speedMax); // 10%=0.35"; 100%=3.5"
        // Compute x & y components required to move toward point (with angle correction)
        double relative_x_to_point = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relative_y_to_point = Math.sin(deltaAngleToPoint) * distanceToPoint;
        // Compute absolute-value x and y distances for scaling
        double relative_x_abs = Math.abs( relative_x_to_point );
        double relative_y_abs = Math.abs( relative_y_to_point );
        // Compute full movement power that preserves the shape/ratios of the intended movement direction
        double movement_x_power = (relative_x_to_point / (relative_y_abs + relative_x_abs)) * speedMax;
        double movement_y_power = (relative_y_to_point / (relative_y_abs + relative_x_abs)) * speedMax;
        // Are we looking for SPEED (DRIVE_THRU) or do we need ACCURATE final stopping position (DRIVE_TO)?
        if( driveType == DRIVE_TO ) {
           // At what distance should we start to reduce from full-power driving?
           double x_slowdown_distance = 10.0 * speedMax;  // 10% power =  1  inch; 100% power = 10 inches
           double y_slowdown_distance =  7.5 * speedMax;  // 10% power = 3/4 inch; 100% power = 7.5 inches
           double x_min_power = (relative_x_to_point > 0.0)? 0.10 : -0.10;
           double y_min_power = (relative_y_to_point > 0.0)? 0.10 : -0.10;
           // If we're inside the tolerance then min power can be 0.0
           if( relative_x_abs <= xy_tolerance ) x_min_power = 0.0;
           if( relative_y_abs <= xy_tolerance ) y_min_power = 0.0;
           // Is it time to start applying that slow-down to forward movement?
           if( relative_x_abs <= x_slowdown_distance )
              movement_x_power = (relative_x_to_point/x_slowdown_distance) * speedMax + x_min_power;
           // Is it time to start applying that slow-down to lateral movement?
           if( relative_y_abs <= y_slowdown_distance )
              movement_y_power = (relative_y_to_point/y_slowdown_distance) * speedMax + y_min_power;
        } // DRIVE_TO
        // Compute robot orientation-angle error
        double robot_radian_err = AngleWrapRadians( Math.toRadians(angleTarget) - angle_world );  // radians
        // Determine the proper angle-error tolerance (within this tolerance rotation_power drops to 0%)
        double angle_tolerance = 0.25;  // degrees (not enforced for DRIVE_THRU, but used to retain angle alignment)
        double smallAngleSpeed = 0.09;  // (0.19 .. 0.10) due to min_turn_power
        // Once angle-error is below 15deg, scale-down rotational power based on remaining error, but don't
        // drop below 10% (0.10) until we're inside the angle tolerance (when rotation_power goes to zero).
        double small_rad_error = Math.abs( robot_radian_err / Math.toRadians(15.0) );
        double min_turn_power = (robot_radian_err <= Math.toRadians(angle_tolerance))? 0.00 : 0.10;
        double adjusted_turn_power = (small_rad_error <= 1.0)? (small_rad_error * smallAngleSpeed + min_turn_power) : turnMax;
        double rotation_power = (robot_radian_err > 0.0)? adjusted_turn_power : -adjusted_turn_power;
        // Translate X,Y,rotation power levels into mecanum wheel power values
        // Note that this is the same math used for tele-op robot-centric driving
        // except "x" and "y" for the odometry are opposite that of the controller joystick
        // (assumes right motors are FORWARD; left motors are  REVERSE; positive power is FORWARD)
        double frontRight = movement_x_power - movement_y_power - rotation_power;
        double frontLeft  = movement_x_power + movement_y_power + rotation_power;
        double backRight  = movement_x_power + movement_y_power - rotation_power;
        double backLeft   = movement_x_power - movement_y_power + rotation_power;
        // Determine the maximum motor power
        double maxWheelPower = Math.max( Math.max( Math.abs(backLeft),  Math.abs(backRight)  ),
                                         Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        // Ensure no wheel powers exceeds 1.0 or 100%
        if( maxWheelPower > 1.00 )
        {
            backLeft   /= maxWheelPower;
            backRight  /= maxWheelPower;
            frontLeft  /= maxWheelPower;
            frontRight /= maxWheelPower;
        }
        boolean ODOMETRY_DEBUG = false;
        if( ODOMETRY_DEBUG ) {
            sleep(100);       // allow 0.1 seconds of progress (from prior loop)...
            robot.driveTrainMotorsZero(); // then stop and observe
            telemetry.addData("World X (inches)", "%.2f in", x_world );
            telemetry.addData("World Y (inches)", "%.2f in", y_world );
            telemetry.addData("Orientation (deg)","%.2f deg", Math.toDegrees(angle_world) );
            telemetry.addData("distanceToPoint", "%.2f in", distanceToPoint);
            telemetry.addData("angleToPoint", "%.4f deg", Math.toDegrees(angleToPoint));
            telemetry.addData("deltaAngleToPoint", "%.4f deg", Math.toDegrees(angleToPoint));
            telemetry.addData("relative_x_to_point", "%.2f in", relative_x_to_point);
            telemetry.addData("relative_y_to_point", "%.2f in", relative_y_to_point);
            telemetry.addData("robot_radian_err", "%.4f deg", Math.toDegrees(robot_radian_err));
            telemetry.addData("movement_x_power", "%.2f", movement_x_power);
            telemetry.addData("movement_y_power", "%.2f", movement_y_power);
            telemetry.addData("rotation_power", "%.2f", rotation_power);
            telemetry.update();
            sleep(3500);  // so we can read the output above
        } // ODOMETRY_DEBUG
        // Have we reached the desired target location/orientation?
        if( driveType == DRIVE_TO ) {
           if( (distanceToPoint <= xy_tolerance) && (Math.abs(robot_radian_err) <= Math.toRadians(angle_tolerance)) )
               return true;
        } // DRIVE_TO
        else {
           // NOTE: Depending on the move_power and turn_power specified, and the size of the
           // two changes needed (inches-of-travel & degrees-of-turn), it's possible that one may 
           // achieve the DRIVE_THRU tolerance when the other has not.  We assume the x-y distance
           // is the primary objective (and that we shouldn't STOP and wait for the rotation angle
           // to catch up) so we only confirm compliance to one and not both motion objectives.
           // We define  a tight angle tolerance for DRIVE_THRU so that we still attempt to "maintain"
           // the correct angle (when  possible) during  other DRIVE_THRU movements.
           if( distanceToPoint <= xy_tolerance )
               return true;
        } // DRIVE_THRU
        // NO, WE'RE NOT DONE! Update motor power settings
//      telemetry.addData("distanceToPoint", "%.2f in (x=%.2f y=%.2f)",
//              distanceToPoint, relative_x_to_point, relative_y_to_point );
//      telemetry.addData("motors", "%.2f %.2f %.2f %.2f",
//              frontLeft, frontRight, backLeft, backRight );
//      telemetry.update();
        robot.driveTrainMotors( frontLeft, frontRight, backLeft, backRight );
        return false;
    } // moveToPosition

    /**
     * Ensure angle is in the range of -PI to +PI (-180 to +180 deg)
     * @param angleRadians
     * @return
     */
    public double AngleWrapRadians( double angleRadians ){
        while( angleRadians < -Math.PI ) {
            angleRadians += 2.0*Math.PI;
        }
        while( angleRadians > Math.PI ){
            angleRadians -= 2.0*Math.PI;
        }
        return angleRadians;
    }

    /**
     * Ensure angle is in the range of -180 to +180 deg (-PI to +PI)
     * @param angleDegrees
     * @return
     */
    public double AngleWrapDegrees( double angleDegrees ){
        while( angleDegrees < -180 ) {
            angleDegrees += 360.0;
        }
        while( angleDegrees > 180 ){
            angleDegrees -= 360.0;
        }
        return angleDegrees;
    } // AngleWrapDegrees
    
    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        int leftChange  = robot.leftOdometerCount  - robot.leftOdometerPrev;
        int rightChange = robot.rightOdometerCount - robot.rightOdometerPrev;
        //Calculate Angle
        double changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians += changeInRobotOrientation;
        robotOrientationRadians = AngleWrapRadians( robotOrientationRadians );   // Keep between -PI and +PI
        //Get the components of the motion
        int rawHorizontalChange = robot.strafeOdometerCount - robot.strafeOdometerPrev;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);
        double p = ((rightChange + leftChange) / 2.0);
        double n = horizontalChange;
        //Calculate and update the position values
        robotGlobalXCoordinatePosition += (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition += (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));
    } // globalCoordinatePositionUpdate

    /*--------------------------------------------------------------------------------------------*/
    // Resets odometry global position to 0/0/0 and ensures zero accumulated encoder counts
    public void globalCoordinatePositionReset(){
        // Read starting odomentry counts (so "prev" becomes "this" on next cycle)
        robot.readBulkData();
        // Zero our starting position (x/y & angle) 
        robotOrientationRadians        = 0.0;   // 0deg (straight forward)
        robotGlobalXCoordinatePosition = 0.0;   // in odometer counts
        robotGlobalYCoordinatePosition = 0.0;
    } // globalCoordinatePositionReset

} // AutonomousBase
