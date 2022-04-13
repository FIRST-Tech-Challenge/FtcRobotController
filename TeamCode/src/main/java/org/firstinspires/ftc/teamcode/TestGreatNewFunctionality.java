/* FTC Team 7572 - Version 2.0 (12/10/2021)
*/
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwareBothHubs.MIN_DRIVE_POW;
import static org.firstinspires.ftc.teamcode.HardwareBothHubs.MIN_STRAFE_POW;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    int leftRange, rightRange, frontRange, backRange = 0;
    double robotAngle = 0.0;

    /*---------------------------------------------------------------------------------------------
     * getError determines the error between the target angle and the robot's current heading
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
    } // getError()


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
        robot.readBulkData();
        double driveAngle = robotAngle = robot.headingIMU();
        double angleError;
        double rotatePower;
        double drivePower;
        double fl, fr, bl, br;
        double scaleFactor;
        timer.reset();
        while(opModeIsActive() && !reachedDestination && (timer.milliseconds() < timeout)) {
            robot.readBulkData();
            robot.headingIMU();
            sensorDistance = leftWall ? (leftRange = robot.singleSonarRangeL()) : (rightRange = robot.singleSonarRangeR());
            distanceError = sensorDistance - distanceFromWall;
            if(Math.abs(distanceError) > allowedError) {
                // Right is negative angle, left positive
                angleError = -getAngleError(driveAngle);
                rotatePower = angleError * angleErrorMult;
                drivePower = distanceError * distanceErrorMult;
                drivePower = leftWall ? drivePower : -drivePower;
                drivePower = Math.copySign(Math.min(Math.max(Math.abs(drivePower), MIN_STRAFE_POW), maxPower), drivePower);
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
        robot.readBulkData();
        double driveAngle = robotAngle = robot.headingIMU();
        double angleError;
        double rotatePower;
        double drivePower;
        double fl, fr, bl, br;
        double scaleFactor;
        timer.reset();
        while(opModeIsActive() && !reachedDestination && (timer.milliseconds() < timeout)) {
            robot.readBulkData();
            robot.headingIMU();
            sensorDistance = frontWall ? (frontRange = robot.singleSonarRangeF()) : (backRange = robot.singleSonarRangeB());
            distanceError = sensorDistance - distanceFromWall;
            if(Math.abs(distanceError) > allowedError) {
                angleError = -getAngleError(driveAngle);
                rotatePower = angleError * angleErrorMult;
                drivePower = distanceError * distanceErrorMult;
                drivePower = frontWall ? drivePower : -drivePower;
                drivePower = Math.copySign(Math.min(Math.max(Math.abs(drivePower), MIN_DRIVE_POW), maxPower), drivePower);
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
        if(!reachedDestination) {
            robot.driveTrainMotorsZero();
        }

        return reachedDestination;
    } // driveToWall

    @Override
    public void runOpMode() throws InterruptedException {
        boolean reachedDestination = false;
        double driveSpeed = 0.0;
        double strafeSpeed = 0.0;
        double turnSpeed = 0.0;
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

            if(gamepad1_dpad_up_now && !gamepad1_dpad_up_last) {
                reachedDestination = driveToWall(true, 0.75, 30, 5000);
            }
            if(gamepad1_dpad_down_now && !gamepad1_dpad_down_last) {
                reachedDestination = driveToWall(false, 0.75, 30, 5000);
            }
            if(gamepad1_dpad_left_now && !gamepad1_dpad_left_last) {
                reachedDestination = strafeToWall(true, 0.75, 30, 5000);
//                turnSpeed -= 0.05;
//                robot.driveTrainMotors(turnSpeed, -turnSpeed, turnSpeed, -turnSpeed);
            }
            if(gamepad1_dpad_right_now && !gamepad1_dpad_right_last) {
//                turnSpeed += 0.05;
//                robot.driveTrainMotors(turnSpeed, -turnSpeed, turnSpeed, -turnSpeed);
                reachedDestination = strafeToWall(false, 0.75, 30, 5000);
            }

            telemetry.addData("Reached Destination: ", reachedDestination ? "true" : "false");
            telemetry.addData("Range L/R", "L %d R %d", leftRange, rightRange);
            telemetry.addData("Range F/B", "F %d B %d", frontRange, backRange);
            telemetry.addData("Drive Angle: ", robotAngle);
            telemetry.addData("Heading: ", robot.headingAngle);
            telemetry.addData("Turn Speed: ", turnSpeed);
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
