package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import java.util.EnumMap;

public class DriveSystem {

    public enum MotorNames {
        FRONTLEFT, FRONTRIGHT, BACKRIGHT, BACKLEFT
    }

    public enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT;

        private static boolean isStrafe(Direction direction) {
            return direction == LEFT || direction == RIGHT;
        }
    }

    public static final double SLOW_DRIVE_COEFF = 0.4;
    // Gives the point at which to switch to less than full power
    public static final double FULL_POWER_UNTIL = 30;
    // Minimum speed to complete the turn
    public static final double MIN_SPEED = 0.37;
    // 12.6 inches circumference of a wheel
    // 319 mm circumference of a wheel
    // 1120 ticks in a revolution
    // 1120 / 319 = 3.51
    private final double TICKS_IN_MM = 3.51;
    public static final double STRAFE_COEFF = 0.09;
    public static final String TAG = "DriveSystem";
    public static final double P_TURN_COEFF = 0.012;     // Larger is more responsive, but also less stable
    public static final double HEADING_THRESHOLD = 1 ;      // As tight as we can make it with an integer gyro

    public EnumMap<MotorNames, DcMotor> motors;

    public IMUSystem imuSystem;

    private int mTargetTicks;
    private double mTargetHeading;
    public boolean mSlowDrive;

    /**
     * Handles the data for the abstract creation of a drive system with four wheels
     */
    public DriveSystem(EnumMap<MotorNames, DcMotor> motors, BNO055IMU imu) {
        this.motors = motors;
        mTargetTicks = 0;
        initMotors();
        imuSystem = new IMUSystem(imu);
    }

    public DriveSystem(EnumMap<MotorNames, DcMotor> motors) {
        this.motors = motors;
        mTargetTicks = 0;
        initMotors();
    }

    /**
     * Set the power of the drive system
     * @param power power of the system
     */
    public void setMotorPower(double power) {
        for (DcMotor motor : motors.values()) {
            motor.setPower(power);
        }
    }

    public void initMotors() {
        motors.forEach((name, motor) -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void slowDrive(boolean slowDrive) {
        mSlowDrive = slowDrive;
    }

    private void setDriveSpeed(DcMotor motor, double motorPower) {
        motor.setPower(Range.clip(mSlowDrive ?
                SLOW_DRIVE_COEFF * motorPower : motorPower, -1, 1));
    }

    /**
     * Clips joystick values and drives the motors.
     * @param rightX Right X joystick value
     * @param leftX Left X joystick value
     * @param leftY Left Y joystick value in case you couldn't tell from the others
     */
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

        double frontLeftPower = -leftY + rightX + leftX;
        double frontRightPower = -leftY - rightX - leftX;
        double backLeftPower = -leftY + rightX - leftX;
        double backRightPower = -leftY - rightX + leftX;



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

    public boolean driveToPositionTicks(int ticks, Direction direction, double maxPower) {
        if(mTargetTicks == 0) {
            driveToPositionInit(ticks, direction, maxPower);
        }
        for (DcMotor motor : motors.values()) {
            int offset = Math.abs(motor.getCurrentPosition() - mTargetTicks);
            if(offset <= 15){
                // Shut down motors
                // Reset target
                stopAndReset();
                // Motor has reached target
                return true;
            }
        }

        if (Direction.isStrafe(direction)) {
            double diff = computeDegreesDiff();
            double correction = Range.clip(STRAFE_COEFF * diff, -1, 1);
            int sign = direction == Direction.LEFT ? -1 : 1;
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

    private void driveToPositionInit(int ticks, Direction direction, double maxPower) {
        mTargetTicks = direction == Direction.BACKWARD ? -ticks : ticks;
        motors.forEach((name, motor) -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if(Direction.isStrafe(direction)) {
                strafeInit();
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
            } else {
                motor.setTargetPosition(mTargetTicks);
            }
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(maxPower);
        });
    }

    public void stopAndReset() {
        setMotorPower(0.0);
        mTargetTicks = 0;
        mTargetHeading = 0;
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void strafeInit() {
        mTargetHeading = imuSystem.getHeading();
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        for (DcMotor motor : motors.values()) {
            motor.setMode(runMode);
        }
    }

    public boolean driveToPosition(int millimeters, Direction direction, double maxPower) {
        return driveToPositionTicks(millimetersToTicks(millimeters), direction, maxPower);
    }

    /**
     * Converts millimeters to ticks
     * @param millimeters Millimeters to convert to ticks
     * @return number of ticks
     */
    private int millimetersToTicks(int millimeters) {
        return (int) Math.round(millimeters * TICKS_IN_MM);
    }

    /**
     * Turns relative the heading upon construction
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     */
    public boolean turnAbsolute(double degrees, double maxPower) {
        // Since it is vertical, use pitch instead of heading
        return turn(diffFromAbs(degrees), maxPower);
    }

    /**
     * Turns the robot by a given number of degrees
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     */
    public boolean turn(double degrees, double maxPower) {
        // Since controller hub is vertical, use pitch instead of heading
        double heading = imuSystem.getHeading();
        // if controller hub is flat: double heading = imuSystem.getHeading();
        if(mTargetHeading == 0) {
            mTargetHeading = (heading + degrees) % 360;
            Log.d(TAG, "Setting Heading -- Target: " + mTargetHeading);

            Log.d(TAG, "Degrees: " + degrees);
        }
        double difference = mTargetHeading - heading;
        Log.d(TAG,"Difference: " + difference);
        return onHeading(maxPower, heading);

    }

    /**
     * Perform one cycle of closed loop heading control.
     * @param speed Desired speed of turn
     */
    public boolean onHeading(double speed, double heading) {
        double leftSpeed;

        // determine turn power based on +/- error
        double error = computeDegreesDiff();
        Log.d(TAG, "Error: " + error);

        // If it gets there: stop
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            mTargetHeading = 0;
            setMotorPower(0);
            return true;
        }

        // Go full speed until 60% there
        leftSpeed = Math.abs(error) / 125.0;

        Log.d(TAG, "Left Speed: " + leftSpeed);
        leftSpeed = Range.clip(leftSpeed, MIN_SPEED, 1.0);

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
        // calculate error in -179 to +180 range
        // When vertical use pitch instead of heading
        double robotDiff = heading - imuSystem.getHeading();
        Log.d(TAG,"Difference from initial: " + robotDiff);
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
     * @return
     */
    private double getSteer(double error) {
        return Range.clip(error *  P_TURN_COEFF, -1, 1);
    }

    /**
     * Causes the system to tank drive
     * @param leftPower sets the left side power of the robot
     * @param rightPower sets the right side power of the robot
     */
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

    /**
     * computeDegreesDiff determines the error between the target angle and the robot's current heading
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double computeDegreesDiff() {
        double diff = mTargetHeading - imuSystem.getHeading();
        return Math.abs(diff) == 180 ? diff : diff % 180;
    }

}