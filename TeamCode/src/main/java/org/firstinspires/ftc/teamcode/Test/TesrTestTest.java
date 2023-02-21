

package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


    @Autonomous(name="RRIMU2")
@Disabled
    public class TesrTestTest extends LinearOpMode {


        /* Declare OpMode members. */
        private DcMotor frontleft;
        private DcMotor frontright;
        private DcMotor backleft;
        private DcMotor backright;
        private DcMotor lift;
//        private CRServo Grabbie;
        private BNO055IMU imu         = null;      // Control/Expansion Hub IMU

        private double          robotHeading  = 0;
        private double          headingOffset = 0;
        private double          headingError  = 0;

        Orientation angles;
        Acceleration gravity;

        // These variable are declared here (as class members) so they can be updated in various methods,
        // but still be displayed by sendTelemetry()
        private double  targetHeading = 0;
        private double  driveSpeed    = 0;
        private double  turnSpeed     = 0;
        private double  leftSpeed     = 0;
        private double  rightSpeed    = 0;
        private int     frontleftTarget    = 0;
        private int     frontrightTarget   = 0;
        private int     backleftTarget    = 0;
        private int     backrightTarget   = 0;


        ModernRoboticsI2cGyro gyro;

        double amountError = 2;

        static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
        static final double P_DRIVE_COEFF = 0.07;     // Larger is more responsive, but also less stable


        Integer cpr = 570; //counts per rotation
        Integer gearRatio = 1;
        Double diameter = 4.125;
        Double cpi = (cpr * gearRatio)/(Math.PI * diameter); //counts per inch
        Double bias = 0.8; //default 0.8
        Double messyBias = 0.9; //change to adjust only strafing movements

        Double conversions = cpi * bias;
        Boolean exit = false;

        // Calculate the COUNTS_PER_INCH for your specific drive train.
        // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
        // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
        // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
        // This is gearing DOWN for less speed and more torque.
        // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
        Integer COUNTS_PER_MOTOR_REV = 570; //counts per rotation
        Integer DRIVE_GEAR_REDUCTION = 1;
        Double WHEEL_DIAMETER_INCHES = 4.125;
        Double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(Math.PI * WHEEL_DIAMETER_INCHES); //counts per inch



        // These constants define the desired driving/control characteristics
        // They can/should be tweaked to suit the specific robot drive train.
        static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
        static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
        static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
        // Define the Proportional control coefficient (or GAIN) for "heading control".
        // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
        // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
        // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
        static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
        static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable


        @Override
        public void runOpMode() {

            // Initialize the drive system variables.
            frontleft = hardwareMap.get(DcMotor.class, "frontleft");
            frontright = hardwareMap.get(DcMotor.class, "frontright");
            backleft = hardwareMap.get(DcMotor.class, "backleft");
            backright = hardwareMap.get(DcMotor.class, "backright");
            lift = hardwareMap.get(DcMotor.class, "lift");
//            Grabbie = hardwareMap.get(CRServo.class, "Grabbie");

            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            frontleft.setDirection(DcMotor.Direction.REVERSE);
            frontright.setDirection(DcMotor.Direction.FORWARD);
            backleft.setDirection(DcMotor.Direction.REVERSE);
            backright.setDirection(DcMotor.Direction.FORWARD);

            // define initialization values for IMU, and then initialize it.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
            frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Wait for the game to start (Display Gyro value while waiting)
            while (opModeInInit()) {
                telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
                telemetry.update();
            }

            // Set the encoders for closed loop speed control, and reset the heading.
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetHeading();

            waitForStart();

           IMUTurn(.5,180);
           IMUHold(.5,180,2);
           Strafe(.5,10);
           goLift(10,.5);




        }
        /*
         * ====================================================================================================
         * Driving "Helper" functions are below this line.
         * These provide the high and low level methods that handle driving straight and turning.
         * ====================================================================================================
         */

        // **********  HIGH Level driving functions.  ********************

        public void goLift(double inches, double speed){

            telemetry.addData("Encodercount", lift.getCurrentPosition() );
            telemetry.update();
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            int move =  -(int)(Math.round(inches*93.3));

            lift.setTargetPosition(lift.getCurrentPosition() + move);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(speed);
        }

        /**
         *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
         *  Move will stop if either of these conditions occur:
         *  1) Move gets to the desired position
         *  2) Driver stops the opmode running.
         *
         * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
         * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
         * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
         *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
         *                   If a relative angle is required, add/subtract from the current robotHeading.
         */
        public void IMUDrive(double maxDriveSpeed,
                             double distance,
                             double heading) {


            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                int moveCounts = (int)(distance * COUNTS_PER_INCH);
                frontleftTarget = frontleft.getCurrentPosition() + moveCounts;
                frontrightTarget = frontright.getCurrentPosition() + moveCounts;
                backleftTarget = backleft.getCurrentPosition() + moveCounts;
                backrightTarget = backright.getCurrentPosition() + moveCounts;

                // Set Target FIRST, then turn on RUN_TO_POSITION
                frontleft.setTargetPosition(frontleftTarget);
                frontright.setTargetPosition(frontrightTarget);
                backleft.setTargetPosition(backleftTarget);
                backright.setTargetPosition(backrightTarget);

                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set the required driving speed  (must be positive for RUN_TO_POSITION)
                // Start driving straight, and then enter the control loop
                maxDriveSpeed = Math.abs(maxDriveSpeed);
                moveRobot(maxDriveSpeed, 0);

                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() &&
                        (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy())) {

                    // Determine required steering to keep on heading
                    turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        turnSpeed *= -1.0;

                    // Apply the turning correction to the current driving speed.
                    moveRobot(driveSpeed, turnSpeed);

                    // Display drive status for the driver.
                    sendTelemetry(true);

                }


                // Stop all motion & Turn off RUN_TO_POSITION
                moveRobot(0, 0);
//            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        /**
         *  Method to spin on central axis to point in a new direction.
         *  Move will stop if either of these conditions occur:
         *  1) Move gets to the heading (angle)
         *  2) Driver stops the opmode running.
         *
         * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
         * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
         *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
         *              If a relative angle is required, add/subtract from current heading.
         */
        public void IMUTurn(double maxTurnSpeed, double heading) {


            // Run getSteeringCorrection() once to pre-calculate the current error
            getSteeringCorrection(heading, P_DRIVE_GAIN);

            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

                // Clip the speed to the maximum permitted value.
                turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

                // Pivot in place by applying the turning correction
                moveRobot(0, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(false);
            }

            // Stop all motion;
//        moveRobot(0, 0);
            backright.setPower(0);
            backleft.setPower(0);
            frontright.setPower(0);
            frontleft.setPower(0);

        }

        /**
         *  Method to obtain & hold a heading for a finite amount of time
         *  Move will stop once the requested time has elapsed
         *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
         *
         * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
         * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
         *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
         *                   If a relative angle is required, add/subtract from current heading.
         * @param holdTime   Length of time (in seconds) to hold the specified heading.
         */
        public void IMUHold(double maxTurnSpeed, double heading, double holdTime) {

            ElapsedTime holdTimer = new ElapsedTime();
            holdTimer.reset();

            // keep looping while we have time remaining.
            while (opModeIsActive() && (holdTimer.time() < holdTime)) {
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

                // Clip the speed to the maximum permitted value.
                turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

                // Pivot in place by applying the turning correction
                moveRobot(0, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(false);
            }

            // Stop all motion;
            moveRobot(0, 0);
        }

        // **********  LOW Level driving functions.  ********************

        /**
         * This method uses a Proportional Controller to determine how much steering correction is required.
         *
         * @param desiredHeading        The desired absolute heading (relative to last heading reset)
         * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
         * @return                      Turning power needed to get to required heading.
         */
        public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
            targetHeading = desiredHeading;  // Save for telemetry

            // Get the robot heading by applying an offset to the IMU heading
            robotHeading = getRawHeading() - headingOffset;

            // Determine the heading current error
            headingError = targetHeading - robotHeading;

            // Normalize the error to be within +/- 180 degrees
            while (headingError > 180)  headingError -= 360;
            while (headingError <= -180) headingError += 360;

            // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
            return Range.clip(headingError * proportionalGain, -1, 1);
        }

        /**
         * This method takes separate drive (fwd/rev) and turn (right/left) requests,
         * combines them, and applies the appropriate speed commands to the left and right wheel motors.
         * @param drive forward motor speed
         * @param turn  clockwise turning motor speed.
         */
        public void moveRobot(double drive, double turn) {

            driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
            turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

            leftSpeed  = drive - turn;
            rightSpeed = drive + turn;

            // Scale speeds down if either one exceeds +/- 1.0;
            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            frontleft.setPower(leftSpeed);
            frontright.setPower(rightSpeed);
            backleft.setPower(leftSpeed);
            backright.setPower(rightSpeed);
        }

        /**
         *  Display the various control parameters while driving
         *
         * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
         */
        private void sendTelemetry(boolean straight) {

            if (straight) {
                telemetry.addData("Motion", "Drive Straight");
                telemetry.addData("Target Pos L:R",  "%7d:%7d",      frontleftTarget,  frontrightTarget, backleftTarget, backrightTarget);
                telemetry.addData("Actual Pos L:R",  "%7d:%7d",      frontleft.getCurrentPosition(),
                        frontright.getCurrentPosition(), backleft.getCurrentPosition(), backright.getCurrentPosition());
            } else {
                telemetry.addData("Motion", "Turning");
            }

            telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
            telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
            telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
            telemetry.update();
        }

        /**
         * read the raw (un-offset Gyro heading) directly from the IMU
         */
        public double getRawHeading() {
            Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return angles.firstAngle;
        }

        /**
         * Reset the "offset" heading back to zero
         */
        public void resetHeading() {
            // Save a new heading offset equal to the current raw heading.
            headingOffset = getRawHeading();
            robotHeading = 0;
        }
        public double devertify(double degrees) {
            if (degrees < 0) {
                degrees = degrees + 360;
            }
            return degrees;
        }

        public double convertify(double degrees) {
            if (degrees > 179) {
                degrees = -(360 - degrees);
            } else if (degrees < -180) {
                degrees = 360 + degrees;
            } else if (degrees > 360) {
                degrees = degrees - 360;
            }
            return degrees;
        }




        public void Strafe (double power,double inches)
        {
            int move = (int)(Math.round(inches * conversions));

            frontleft.setTargetPosition(frontleft.getCurrentPosition() - move);
            frontright.setTargetPosition(frontright.getCurrentPosition() + move);
            backleft.setTargetPosition(backleft.getCurrentPosition() + move);
            backright.setTargetPosition(backright.getCurrentPosition() -move);

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleft.setPower(-power);
            frontright.setPower(-power);
            backright.setPower(-power);
            backleft.setPower(-power);

            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && frontleft.isBusy())
            {
                if(exit)
                {
                    backleft.setPower(0);
                    frontleft.setPower(0);
                    frontright.setPower(0);
                    backright.setPower(0);
                }
            }
        }

        public void MoveLift (double power, double inches)

        {
            int move = (int)(Math.round(inches * conversions));

            lift.setTargetPosition(lift.getCurrentPosition() + move);

            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift.setPower(power);

            while (lift.isBusy())
            {
                if(exit)
                {
                    lift.setPower(0);
                }
            }
        }

    }

