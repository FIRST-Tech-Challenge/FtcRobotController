package org.firstinspires.ftc.teamcode.components;

import android.os.Build;
import android.util.Log;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.params.DriveParams;

import java.util.EnumMap;

public class DriveSystem {

    /********************************** ENUMS **************************************************/
    public enum MotorNames {
        FRONTLEFT, FRONTRIGHT, BACKRIGHT, BACKLEFT
    }

    public enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT;

        private static boolean isStrafe(Direction direction) {
            return direction == LEFT || direction == RIGHT;
        }
    }

    /*********************** HARDWARE CONSTANTS **************************************************/
    private final double TICKS_IN_MM = ticksInMm();
    // Logging tag
    public static final String TAG = "DriveSystem";

    /******************************** Systems **************************************************/
    public EnumMap<MotorNames, DcMotor> motors;
    public IMUSystem imuSystem;

    // Target position to run to when driving to position
    private int mTargetTicks;
    // Target heading to run to when turning
    private double mTargetHeading;
    // Are we slow-driving?
    public boolean mSlowDrive;

    /******************************** INIT **************************************************/

    /**
     * Handles the data for the abstract creation of a drive system with four wheels
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public DriveSystem(EnumMap<MotorNames, DcMotor> motors, BNO055IMU imu) {
        this.motors = motors;
        mTargetTicks = 0;
        initMotors();
        imuSystem = new IMUSystem(imu);
    }

    /**
     * Handles the data for the abstract creation of a drive system with four wheels without IMU
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public DriveSystem(EnumMap<MotorNames, DcMotor> motors) {
        this.motors = motors;
        mTargetTicks = 0;
        initMotors();
    }

    /** Initializes motors
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void initMotors() {
        motors.forEach((name, motor) -> {
            // Reset encoders
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Set motor directions to drive forwards
            switch(name) {
                case FRONTLEFT:
                case BACKLEFT:
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                    break;
                case FRONTRIGHT:
                case BACKRIGHT:
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                    break;
            }
        });
        // setMotorPower(0);
    }

    /******************************** LOW LEVEL CONTROL *******************************************/

    /**
     * Set the power of the drive system
     * @param power power of the system
     */
    public void setMotorPower(double power) {
        for (DcMotor motor : motors.values()) {
            motor.setPower(power);
        }
    }

    /**
     * Set the drive speed of given motor to given power depending on whether driving slow or not
     * @param motor given motor
     * @param motorPower power of the system
     */
    private void setDriveSpeed(DcMotor motor, double motorPower) {
        motor.setPower(Range.clip(mSlowDrive ?
                DriveParams.SLOW_DRIVE_COEFF * motorPower : motorPower, -1, 1));
    }

    // Set whether we are driving slow
    public void slowDrive(boolean slowDrive) {
        mSlowDrive = slowDrive;
    }

    // Sets the run mode of all motors to a given run mode
    public void setRunMode(DcMotor.RunMode runMode) {
        for (DcMotor motor : motors.values()) {
            motor.setMode(runMode);
        }
    }

    // Stops motors and resets the motor's mode, target position and heading
    public void stopAndReset() {
        setMotorPower(0.0);
        mTargetTicks   = 0;
        mTargetHeading = 0;
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*********************************** DRIVE **************************************************/

    /**
     * Teleop Drive
     * Clips joystick values and drives the motors.
     * @param rightX Right X joystick value
     * @param leftX Left X joystick value
     * @param leftY Left Y joystick value in case you couldn't tell from the others
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void drive(float rightX, float leftX, float leftY) {
        // Prevent small values from causing the robot to drift
        if (Math.abs(rightX) < 0.01) {
            rightX = 0.0f;
        }
        if (Math.abs(leftX) < 0.01) {
            leftX = 0.0f;
        }
        if (Math.abs(leftY) < 0.01) {
            leftY = 0.0f;
        }

        double frontLeftPower  = -leftY + rightX + leftX;
        double frontRightPower = -leftY - rightX - leftX;
        double backLeftPower   = -leftY + rightX - leftX;
        double backRightPower  = -leftY - rightX + leftX;

        motors.forEach((name, motor) -> {
            switch(name) {
                case FRONTRIGHT:
                    setDriveSpeed(motor, frontRightPower);
                    break;
                case BACKLEFT:
                    setDriveSpeed(motor, backLeftPower);
                    break;
                case FRONTLEFT:
                    setDriveSpeed(motor, frontLeftPower);
                    break;
                case BACKRIGHT:
                    setDriveSpeed(motor, backRightPower);
                    break;
            }
        });
        mSlowDrive = false;
    }

    /**
     * Drives a given # of ticks in a given direction
     * @param ticks distance in # of ticks
     * @param direction The direction the robot is moving in
     * @param maxPower The maximum power of the motors
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean driveToPositionTicks(int ticks, Direction direction, double maxPower) {
        // Initialize target position
        if(mTargetTicks == 0) {
            driveToPositionInit(ticks, direction, maxPower);
        }
        // Determine distance from desired target and stop if within acceptable tolerance
        for (DcMotor motor : motors.values()) {
            Log.i("MOTOR", motor.getCurrentPosition() + "");
            Log.i("MOTOR_POWER", motor.getPower() + "");
            int offset = Math.abs(motor.getCurrentPosition() - mTargetTicks);
            if (offset <= DriveParams.TICK_TOLERANCE) {
                // Shut down motors and reset target
                stopAndReset();
                // Motor has reached target
                return true;
            }
        }

        if (Direction.isStrafe(direction)) {
            double diff       = computeDegreesDiff();
            double correction = Range.clip(DriveParams.STRAFE_COEFF * diff, -1, 1);
            int sign          = (direction == Direction.LEFT) ? -1 : 1;
            motors.forEach((name, motor) -> {
                switch(name) {
                    case FRONTLEFT:
                    case BACKLEFT:
                        motor.setPower(correction > 0 ? 1 - sign * correction: 1);
                        break;
                    case FRONTRIGHT:
                    case BACKRIGHT:
                        motor.setPower(correction < 0 ? 1 + sign * correction : 1);
                        break;
                }
            });
        }
        // Motor has not reached target
        return false;
    }

    /**
     * Initialize target position for ticks and motor mode
     * @param ticks target distance in ticks
     * @param direction Which way is the robot moving
     * @param maxPower The maximum power of the motors
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    private void driveToPositionInit(int ticks, Direction direction, double maxPower) {
        mTargetTicks = direction == Direction.BACKWARD ? -ticks : ticks;
        // Set target position for each motor
        motors.forEach((name, motor) -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Strafing:
            if (Direction.isStrafe(direction)) {
                // Set target heading to current heading
                mTargetHeading = DriveParams.IMU_VERT ? imuSystem.getPitch() : imuSystem.getHeading();
                int sign = direction == Direction.LEFT ? -1 : 1;

                switch(name){
                    case FRONTLEFT:
                    case BACKRIGHT:
                        motor.setTargetPosition(sign * mTargetTicks);
                        break;
                    case FRONTRIGHT:
                    case BACKLEFT:
                        motor.setTargetPosition(sign * -mTargetTicks);
                        break;
                }
                // Driving forwards or backwards:
            } else {
                motor.setTargetPosition(mTargetTicks);
            }
            // Change mode so robot drives to target ticks
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(maxPower);
        });
    }

    /** Drive to position, but this time with millimeters
     * @param millimeters sets the distance to run in millimeters
     * @param direction sets which direction to go
     * @param maxPower sets the power to run at
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean driveToPosition(int millimeters, Direction direction, double maxPower) {
        return driveToPositionTicks(millimetersToTicks(millimeters), direction, maxPower);
    }

    /**
     * Causes the system to tank drive
     * @param leftPower sets the left side power of the robot
     * @param rightPower sets the right side power of the robot
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    private void tankDrive(double leftPower, double rightPower) {
        motors.forEach((name, motor) -> {
            switch(name) {
                case FRONTLEFT:
                case BACKLEFT:
                    motor.setPower(leftPower);
                    break;
                case FRONTRIGHT:
                case BACKRIGHT:
                    motor.setPower(rightPower);
                    break;
            }
        });
    }

    /************************************ TURN **************************************************/

    /**
     * Turns relative the initial heading upon construction
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     * @return if on heading
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean turnAbsolute(double degrees, double maxPower) {
        return turn(diffFromAbs(degrees), maxPower);
    }

    /**
     * Turns the robot by a given number of degrees
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     * @return if on heading
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean turn(double degrees, double maxPower) {
        // If controller hub is vertical, use pitch instead of heading
        double heading = DriveParams.IMU_VERT ? imuSystem.getPitch() : imuSystem.getHeading();
        // Initialization of target heading
        if(mTargetHeading == 0) {
            mTargetHeading = (heading + degrees) % 360;
            Log.d(TAG, "Setting Heading -- Target: " + mTargetHeading);
            Log.d(TAG, "Degrees: " + degrees);
        }
        // How far off the target heading are we?
        double difference = mTargetHeading - heading;
        Log.d(TAG,"Difference: " + difference);
        return onHeading(maxPower, heading);

    }

    /**
     * Perform one cycle of closed loop heading control.
     * @param speed Desired speed of turn
     * @param heading current heading
     * @return if it finished its turn
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean onHeading(double speed, double heading) {
        double leftSpeed;

        // determine turn power based on +/- error
        double error = computeDegreesDiff();
        Log.d(TAG, "Error: " + error);

        // If it gets there: stop
        if (Math.abs(error) <= DriveParams.HEADING_THRESHOLD) {
            mTargetHeading = 0;
            setMotorPower(0);
            return true;
        }

        // Go full speed until 60% there
        leftSpeed = Math.abs(error) / DriveParams.FULL_POWER_UNTIL;

        Log.d(TAG, "Left Speed: " + leftSpeed);
        leftSpeed = Range.clip(leftSpeed, DriveParams.MIN_SPEED, 1.0);

        // Send desired speeds to motors.
        tankDrive(leftSpeed * Math.signum(error), -leftSpeed * Math.signum(error));
        Log.d(TAG, "Left Speed Post Tank Drive " + leftSpeed);
        Log.d(TAG, "Left Power" + motors.get(MotorNames.FRONTLEFT).getPower());
        return false;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   heading  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double diffFromAbs(double heading) {
        // When vertical use pitch instead of heading
        double currHeading = DriveParams.IMU_VERT ? imuSystem.getPitch() : imuSystem.getHeading();
        double robotDiff   = heading - currHeading;
        Log.d(TAG,"Difference from initial: " + robotDiff);
        // Clip maximum error to -180 : 180
        while (robotDiff > 180) {
            robotDiff -= 360;
        }
        while (robotDiff <= -180) {
            robotDiff += 360;
        }

        Log.d(TAG,"Difference from initial 2: " + robotDiff);
        return robotDiff;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @return desired steering force
     */
    private double getSteer(double error) {
        return Range.clip(error *  DriveParams.P_TURN_COEFF, -1, 1);
    }

    /***************************    HELPER METHODS    ********************************************/

    /**
     * determines the error between the target angle and the robot's current heading
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double computeDegreesDiff() {
        double diff = mTargetHeading - imuSystem.getHeading();
        return Math.abs(diff) == 180 ? diff : diff % 180;
    }

    // Converts inches to millimeters
    private double inchesToMm(double inch) {
        return inch * 25.4;
    }

    // Currently 3.51
    private double ticksInMm() {
        return DriveParams.TICKS_PER_REV / inchesToMm(DriveParams.CIRCUMFERENCE);
    }

    /**
     * Converts millimeters to ticks
     * @param millimeters Millimeters to convert to ticks
     * @return number of ticks
     */
    private int millimetersToTicks(int millimeters) {
        return (int) Math.round(millimeters * TICKS_IN_MM);
    }

}