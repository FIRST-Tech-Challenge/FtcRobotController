package org.firstinspires.ftc.teamcode.mechanism.chassis;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.SENSITIVITY;
import static org.firstinspires.ftc.teamcode.Constants.SLOW_MODE_SENSITIVITY;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MecanumChassis extends Chassis {
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    //imu:
    public BNO055IMU imu;

    Orientation angles;
    Orientation lastAngles = new Orientation();
    public double globalAngle;

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set all motors to 0 power for initialization
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //IMU Setup:
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Define and Initialize BNO055IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void run(Gamepad gamepad){
        //This works, just trust me on it. Slack me or something if you need a full explanation.
        double flPower = (gamepad.left_stick_x - gamepad.left_stick_y + gamepad.right_stick_x)* DRIVE_STICK_THRESHOLD;
        double frPower = (-gamepad.left_stick_x - gamepad.left_stick_y - gamepad.right_stick_x)* DRIVE_STICK_THRESHOLD;
        double blPower = (-gamepad.left_stick_x - gamepad.left_stick_y + gamepad.right_stick_x)* DRIVE_STICK_THRESHOLD;
        double brPower = (gamepad.left_stick_x - gamepad.left_stick_y - gamepad.right_stick_x)* DRIVE_STICK_THRESHOLD;

        //This bit seems complicated, but it just gets the maximum absolute value of all the motors.
        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));

        //If maxPower is less than 1, make it 1. This allows for slower movements.
        maxPower = Math.max(maxPower, 1);

        //Make all of them proportional to the greatest value and factor in the sensitivity.
        flPower = (flPower / maxPower) * SENSITIVITY;
        frPower = (frPower / maxPower) * SENSITIVITY;
        blPower = (blPower / maxPower) * SENSITIVITY;
        brPower = (brPower / maxPower) * SENSITIVITY;

        //Actually set them

        if (gamepad.left_trigger > TRIGGER_THRESHOLD) {
            frontLeft.setPower(flPower* SLOW_MODE_SENSITIVITY);
            frontRight.setPower(frPower* SLOW_MODE_SENSITIVITY);
            backLeft.setPower(blPower* SLOW_MODE_SENSITIVITY);
            backRight.setPower(brPower* SLOW_MODE_SENSITIVITY);
        }
        else {
            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
        }

    }

    // Movement functions
    public void delay(int time) {
        double startTime = elapsedTime.milliseconds();
        while (elapsedTime.milliseconds() - startTime < time) {
        }
    }

    public void moveForwardByTime(double power, int time) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveBackwardByTime(double power, int time) {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeRightByTime(double power, int time){
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeLeftByTime(double power, int time){
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnRightByTime(double power, int time){
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnLeftByTime(double power, int time){
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeRightWithEncoders(double power, int count){
        int start = frontLeft.getCurrentPosition();
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        while(frontLeft.getCurrentPosition() < start + count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeLeftWithEncoders(double power, int count){
        int start = frontRight.getCurrentPosition();
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        while(frontRight.getCurrentPosition() < start + count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveBackwardWithEncoders(double power, int count){
        int start = frontLeft.getCurrentPosition();
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
        while(frontLeft.getCurrentPosition() > start - count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveForwardWithEncoders(double power, int count){
        int start = frontLeft.getCurrentPosition();
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        while(frontLeft.getCurrentPosition() < start + count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnLeftWithEncoders(double power, int count){
        int start = frontRight.getCurrentPosition();
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        while(frontRight.getCurrentPosition() < start + count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnRightWithEncoders(double power, int count){
        int start = frontLeft.getCurrentPosition();
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        while(frontLeft.getCurrentPosition() < start + count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    //IMU Functions:

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     * @param power   Motor power during turn (should not be negative)
     */
    public void rotate(double degrees, double power) throws InterruptedException {

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            power = -power;

        } else if (degrees > 0) {   // turn left.
            //return; //power = power; dont need to change anything because assuming the power is already positive, that should make the robot turn right when put into the turn function
        } else return;

        // set power to rotate.
        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(-power);
        backLeft.setPower(-power);

        double tempPower;
        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {
            }

            while (getAngle() > degrees) {
                tempPower = power * (getAngle() - degrees) / degrees * -1 - 0.1;
                frontRight.setPower(tempPower);
                backRight.setPower(tempPower);
                frontLeft.setPower(-tempPower);
                backLeft.setPower(-tempPower);
            }
        } else {  // left turn.
            while (getAngle() < degrees) {
                tempPower = power * (degrees - getAngle()) / degrees + 0.1;
                frontRight.setPower(tempPower);
                backRight.setPower(tempPower);
                frontLeft.setPower(-tempPower);
                backLeft.setPower(-tempPower);
            }
        }
        // turn the motors off.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        Thread.sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }

    /**
     * Rotates to a global heading using IMU
     *
     * @param degrees
     * @param power
     * @throws InterruptedException
     */
    public void rotateToGlobalAngle(int degrees, double power) throws InterruptedException {

        // get the initial angle of rotation.
        double initialAngle = getGlobalAngle();
        double deltaDegrees = degrees - initialAngle;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (deltaDegrees < 0) {   // turn right.
            power = -power;
        }

        // set power to rotate.
        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(-power);
        backLeft.setPower(-power);

        double tempPower;
        // rotate until turn is completed.
        if (deltaDegrees < 0) {
            // right turn
            while (getGlobalAngle() > degrees && degrees - getGlobalAngle() >= deltaDegrees - 10) {
                tempPower = power * (getGlobalAngle() - degrees) / deltaDegrees + 0.1;
                frontRight.setPower(-tempPower);
                backRight.setPower(-tempPower);
                frontLeft.setPower(tempPower);
                backLeft.setPower(tempPower);
            }
        } else {
            // left turn.
            while (getGlobalAngle() < degrees && degrees - getGlobalAngle() <= deltaDegrees + 10) {
                tempPower = power * (degrees - getGlobalAngle()) / deltaDegrees + 0.1;
                frontRight.setPower(tempPower);
                backRight.setPower(tempPower);
                frontLeft.setPower(-tempPower);
                backLeft.setPower(-tempPower);
            }
        }

        // turn the motors off.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        Thread.sleep(100);

        // reset angle tracking on new heading.
        resetAngle();

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Get current heading relative to intialization angle (i think)
     *
     * @return Current heading / angle from -180 to 180
     */
    public double getGlobalAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}
