/**
 * IMURobot
 *
 * Library for using the IMU with our Robot
 *
 * 30 March 2019
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMURobot {

    //Declare motors that will be used
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public CRServo intake;
    public DcMotor outtakeLeft;
    public DcMotor outtakeRight;

    private CRServo leftConveyor;
    private CRServo rightConveyor;
    private CRServo elevator;

    //Declare servos
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private Servo flipper;

    //Declare the IMU
    private BNO055IMU imu;

    //Orientations are the sets of angles in the three planes
    private Orientation angles, lastAngles, startAngles;
    private double globalAngle;

    //Declare and initialize gain to be used for straight driving
    private double gain = 0.01;
    double stopCont = 0;

    //Declare an opmode and a telemetry object
    private LinearOpMode opMode;
    private Telemetry telemetry;

    private final double SECONDS_PER_CM = 0.012345;
    private final double TICKS_PER_CM =34.225;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = 307.699557;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below

    /**
     *
     * @param motorFrontRight
     * @param motorFrontLeft
     * @param motorBackRight
     * @param motorBackLeft
     * @param imu
     * @param leftIntakeServo
     * @param rightIntakeServo
     * @param leftConveyor
     * @param rightConveyor
     * @param elevator
     * @param flipper
     * @param intake
     * @param outtakeRight
     * @param outtakeLeft
     * @param opMode The Op Mode using the IMURobot object;
     *                for access to the methods opModeIsActive, the exception InterruptedException, and telemetry
     */

    OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);

    public IMURobot(DcMotor motorFrontRight, DcMotor motorFrontLeft, DcMotor motorBackRight, DcMotor motorBackLeft,
                    BNO055IMU imu, CRServo leftIntakeServo, CRServo rightIntakeServo,
                    CRServo leftConveyor, CRServo rightConveyor, CRServo elevator, Servo flipper,
                    CRServo intake, DcMotor outtakeRight, DcMotor outtakeLeft, LinearOpMode opMode){
        this.motorFrontRight = motorFrontRight;
        this.motorFrontLeft = motorFrontLeft;
        this.motorBackRight = motorBackRight;
        this.motorBackLeft = motorBackLeft;
        this.imu = imu;
        this.leftIntakeServo = leftIntakeServo;
        this.rightIntakeServo = rightIntakeServo;
        this.leftConveyor = leftConveyor;
        this.rightConveyor = rightConveyor;
        this.elevator = elevator;
        this.flipper = flipper;
        this.intake = intake;
        this.outtakeRight = outtakeRight;
        this.outtakeLeft = outtakeLeft;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    /**
     *
     * @param motorFrontRight
     * @param motorFrontLeft
     * @param motorBackRight
     * @param motorBackLeft
     * @param imu
     * @param opMode The Op Mode using the IMURobot object;
     *                for access to the methods opModeIsActive, the exception InterruptedException, and telemetry
     */

    public IMURobot(DcMotor motorFrontRight, DcMotor motorFrontLeft, DcMotor motorBackRight, DcMotor motorBackLeft,
                    BNO055IMU imu, CRServo leftIntakeServo,
                    CRServo rightIntakeServo, LinearOpMode opMode){
        this.motorFrontRight = motorFrontRight;
        this.motorFrontLeft = motorFrontLeft;
        this.motorBackRight = motorBackRight;
        this.motorBackLeft = motorBackLeft;
        this.imu = imu;
        this.leftIntakeServo = leftIntakeServo;
        this.rightIntakeServo = rightIntakeServo;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    /**
     * Constructor, no flimsy
     * @param motorFrontRight
     * @param motorFrontLeft
     * @param motorBackRight
     * @param motorBackLeft
     * @param imu
     * @param leftIntakeServo
     * @param rightIntakeServo
     * @param opMode
     */
    /* this one is duplicate of one above, and I didn't know if we wanted this still or not
    public IMURobot(DcMotor motorFrontRight, DcMotor motorFrontLeft, DcMotor motorBackRight, DcMotor motorBackLeft,
                    BNO055IMU imu, Servo leftIntakeServo,
                    Servo rightIntakeServo, LinearOpMode opMode){
        this.motorFrontRight = motorFrontRight;
        this.motorFrontLeft = motorFrontLeft;
        this.motorBackRight = motorBackRight;
        this.motorBackLeft = motorBackLeft;
        this.imu = imu;
        this.leftIntakeServo = leftIntakeServo;
        this.rightIntakeServo = rightIntakeServo;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }
*/
    /**
     *
     * @param motorFrontRight The front right motor
     * @param motorFrontLeft The front left motor
     * @param motorBackRight The back right motor
     * @param motorBackLeft The back left motor
     * @param imu The IMU
     * @param opMode The Op Mode using the IMURobot object;
     *               for access to the methods opModeIsActive, the exception InterruptedException, and telemetry
     */
    public IMURobot(DcMotor motorFrontRight, DcMotor motorFrontLeft, DcMotor motorBackRight, DcMotor motorBackLeft,
                    BNO055IMU imu, LinearOpMode opMode){
        this.motorFrontRight = motorFrontRight;
        this.motorFrontLeft = motorFrontLeft;
        this.motorBackRight = motorBackRight;
        this.motorBackLeft = motorBackLeft;
        this.imu = imu;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    /**
     * Get the IMU parameters
     * @return IMU parameters
     */
    private BNO055IMU.Parameters getIMUParameters(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.mode                = BNO055IMU.SensorMode.IMU;

        return parameters;
    }

    /**
     * Set up and initialize the IMU parameters
     */
    private void setIMUParameters(){
        BNO055IMU.Parameters parameters = getIMUParameters();
        imu.initialize(parameters);
    }

    /**
     * Set directions and zero power behaviors for applicable motors, calibrate the IMU
     * @throws InterruptedException if robot is stopped while IMU is still calibrating
     */
    public void setupRobot() throws InterruptedException{
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);


        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        setIMUParameters();

        while (!imu.isGyroCalibrated()) {
            telemetry.addData("IMU", "calibrating...");
            telemetry.update();
            Thread.sleep(50);
        }

        telemetry.addData("IMU", "ready");
        telemetry.update();
    }

    /**
     * Reset all motor encoders
     */
    public void resetEncoders() {
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Reset the angle measurement
     */
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startAngles = lastAngles;
        globalAngle = 0;
    }

    /**
     * Get the change in angle since the last angle reset
     * @return angle in degrees, positive = counterclockwise, negative = clockwise
     */
    private double getAngle(){
        //Get a new angle measurement
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //Get the difference between current angle measurement and last angle measurement
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //Process the angle to keep it within (-180,180)
        //(Once angle passes +180, it will rollback to -179, and vice versa)
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        //Add the change in angle since last measurement (deltaAngle)
        //to the change in angle since last reset (globalAngle)
        globalAngle += deltaAngle;
        //Set last angle measurement to current angle measurement
        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Create telemetry with the angle at last reset, current angle, and the change in angle since last reset
     */
    private void composeAngleTelemetry(){
        telemetry.addData("Start Angle", startAngles.firstAngle);
        telemetry.addData("Current Angle", angles.firstAngle);
        telemetry.addData("Global Angle", globalAngle);
    }

    /**
     * Complete stop, sets all motor powers to 0
     */
    public void completeStop(){
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    /**
     * Robot turns clockwise at a set power in tank drive
     * @param power desired power for motors
     */
    public void turnClockwise(double power){
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }

    /**
     * Robot turns counterclockwise at a set power in tank drive
     * @param power desired power for motors
     */
    public void turnCounterClockwise(double power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(-power);
    }

    /**
     * Make a precise turn using the gyro
     * @param degrees Negative = clockwise, positive = counterclockwise, min = -180, max = 180
     * @param power Power to turn at
     * @throws InterruptedException if robot is stopped
     */
    public void gyroTurn(int degrees, double power) throws InterruptedException{
        //restart angle tracking
        resetAngle();

        if(degrees < 0){
            turnClockwise(power);
        }else if(degrees > 0){
            turnCounterClockwise(power);
        }else{
            return;
        }

        //Rotate until current angle is equal to the target angle
        if (degrees < 0){
            while (opMode.opModeIsActive() && getAngle() > degrees){
                composeAngleTelemetry();
                //display the target angle
                telemetry.addData("Target angle", degrees);
                telemetry.update();
            }
        }else{
            while (opMode.opModeIsActive() && getAngle() < degrees) {
                composeAngleTelemetry();
                //display the target angle
                telemetry.addData("Target angle", degrees);
                telemetry.update();
            }
        }

        completeStop();
        //Wait .5 seconds to ensure robot is stopped before continuing
        Thread.sleep(500);
        resetAngle();
    }

    /**
     * Check if robot is moving in a straight line using gyro sensor.
     * If not, get a power correction using a preset gain.
     * @return correction, positive = correct clockwise, negative = correct counterclockwise
     */
    public double getCorrection(){
        //Get the current angle of the robot
        double angle = getAngle();
        double correction;

        //Use the angle to calculate the correction
        if (angle == 0){
            //If angle = 0, robot is moving straight; no correction needed
            correction = 0;
        }else{
            //If angle != 0, robot is not moving straight
            //Correction is negative angle (to move the robot in the opposite direction)
            //multiplied by gain; the gain is the sensitivity to angle
            //We have determined that .1 is a good gain; higher gains result in overcorrection
            //Lower gains are ineffective
            correction = -angle*gain;
        }
        //Display correction
        //telemetry.addData("Correction", correction);

        return correction;
    }

    /**
     * Tank Drive, with separate powers for left and right side
     * @param leftPower power for left motors
     * @param rightPower power for right motors
     */
    public void tankDrive(double leftPower, double rightPower){
        motorFrontLeft.setPower(leftPower);
        motorFrontRight.setPower(rightPower);
        motorBackLeft.setPower(leftPower);
        motorBackRight.setPower(rightPower);
    }

    /**
     * Tank strafe, can move in any direction, using correction
     * @param leftPower power for front left and back right motors
     * @param rightPower power for front right and back left motors
     * @param correction
     */
    public void correctedTankStrafe(double leftPower, double rightPower, double correction){
        motorFrontLeft.setPower(leftPower + correction);
        motorFrontRight.setPower(rightPower - correction);
        motorBackLeft.setPower(rightPower + correction);
        motorBackRight.setPower(leftPower - correction);
    }

    /**
     * Drive straight using the gyro
     * @param power power
     * @param seconds time to run
     * @throws InterruptedException if the robot is stopped
     */
    public void gyroDriveSec(double power, double seconds) throws InterruptedException{
        //restart angle tracking
        resetAngle();

        //create an ElapsedTime object to track the time the robot moves
        ElapsedTime timer = new ElapsedTime();
        //restart time tracking
        timer.reset();

        //drive straight with gyro until timer reaches number of given seconds
        while(timer.seconds() < seconds && opMode.opModeIsActive()){
            //Get a correction
            double correction = getCorrection();
            //Use the correction to adjust robot power so robot drives straight
            tankDrive(power + correction, power - correction);
        }
        completeStop();
        //Wait .5 seconds to ensure robot is stopped before continuing
        Thread.sleep(500);
        resetAngle();
    }

    public void gyroDriveCm(double power, double cm) throws InterruptedException{
        gyroDriveSec(power, (cm*SECONDS_PER_CM)/power);
    }

    /**
     * Drives forward using encoders and gyro (uses gyroStrafe)
     * @param power
     * @param cm centimeters
     * @throws InterruptedException if robot is stopped
     */
    public void gyroDriveEncoder(double power, double cm) throws InterruptedException{
        setNewGain(0.05);
        gyroStrafeEncoder(power, 0, cm);
    }

    /**
     * Strafe indefinite amount of time in any direction
     * @param power
     * @param angle Direction to strafe (0 = forward, 180 = backward)
     * @throws InterruptedException if robot is stopped
     */
    public void gyroStrafe(double power, double angle) throws InterruptedException{
        //restart angle tracking
        resetAngle();

        //convert direction (degrees) into radians
        double newDirection = angle * Math.PI/180 + Math.PI/4;
        //calculate powers needed using direction
        double leftPower = Math.cos(newDirection) * power;
        double rightPower = Math.sin(newDirection) * power;

        //while(opMode.opModeIsActive()){
        //Get a correction
        double correction = getCorrection();
        //Use the correction to adjust robot power so robot faces straight
        correctedTankStrafe(leftPower, rightPower, correction);
        //}
    }

    /**
     * Strafe in any direction using gyro to keep robot facing straight forward
     * @param power power
     * @param angle direction to strafe, in degrees (0 = forward, 180 = backward)
     * @param seconds time to run
     * @throws InterruptedException if the robot is stopped
     */
    public void gyroStrafeSec(double power, double angle, double seconds) throws InterruptedException{
        //restart angle tracking
        resetAngle();

        //convert direction (degrees) into radians
        double newDirection = angle * Math.PI/180 + Math.PI/4;
        //calculate powers needed using direction
        double leftPower = Math.cos(newDirection) * power;
        double rightPower = Math.sin(newDirection) * power;

        //create an ElapsedTime object to track the time the robot moves
        ElapsedTime timer = new ElapsedTime();
        //restart time tracking
        timer.reset();

        //strafe using gyro to keep robot facing straight for
        while(timer.seconds() < seconds && opMode.opModeIsActive()){
            //Get a correction
            double correction = getCorrection();
            //Use the correction to adjust robot power so robot faces straight
            correctedTankStrafe(leftPower, rightPower, correction);
        }
        completeStop();
        //Wait .5 seconds to ensure robot is stopped before continuing
        Thread.sleep(500);
        resetAngle();
    }

    public void gyroStrafeCm(double power, double angle, double cm) throws InterruptedException{
        gyroStrafeSec(power, angle, (cm*SECONDS_PER_CM)/power);
    }

    /**
     * Strafe in any direction using encoders.
     * @param power
     * @param angle Direction to strafe (0 = forward, 180 = backward)
     * @param cm
     * @throws InterruptedException if robot is stopped
     */
    public void gyroStrafeEncoder(double power, double angle, double cm) throws InterruptedException{
        double ticks = cm * TICKS_PER_CM;

        //convert direction (degrees) into radians
        double newDirection = angle * Math.PI/180 + Math.PI/4;
        //calculate powers needed using direction
        double leftPower = Math.cos(newDirection) * power;
        double rightPower = Math.sin(newDirection) * power;

        resetEncoders();
        resetAngle();
        setNewGain(0.02);
        while(Math.abs(motorFrontLeft.getCurrentPosition()) < ticks && opMode.opModeIsActive()){
            telemetry.addData("Target ticks", ticks);
            telemetry.addData("Current ticks", Math.abs(motorFrontLeft.getCurrentPosition()));
            telemetry.update();

            double correction = getCorrection();
            correctedTankStrafe(leftPower, rightPower, correction);
        }
        completeStop();
        Thread.sleep(500);
        resetAngle();
        resetEncoders();
    }

    /**
     * Change the gain
     * Good gains should be determined experimentally
     * @param newGain gain (sensitivity): 0 to 0.5 (anything over ~0.5 will cause extreme oscillations in movement)
     */
    public void setNewGain(double newGain){
        gain = newGain;
    }

    /**
     * Get encoder position of the front left motor
     * Can be used in encoder tracking
     * @return Current position of front left motor, in ticks
     */
    public int getMotorPosition() {
        return Math.abs(motorFrontLeft.getCurrentPosition());
    }

    /**
     * Turn on intake
     */

    public void intakeOn() {
        intake.setPower(1);
    }



    /**
     * Turn off intake
     */

    public void intakeOff() {
        intake.setPower(0);
    }




    /**
     * Reverse intake direction
     */

    public void intakeReverse() {
        intake.setPower(-1);
    }

    /*public void releaseIntake() {
        leftIntakeServo.setPosition(1);
        rightIntakeServo.setPosition(0);
    }*/

}
