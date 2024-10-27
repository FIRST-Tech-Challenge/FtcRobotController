/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

public class RobotHardware {

    public static enum MotorDirection {
        FORWARD,
        BACKWARD
    }
    private Blinker control_Hub;
    private IMU imu;

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // naming motors
    private DcMotor rfMotor;
    private DcMotor rbMotor;
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private DcMotor viperSlideMotor;
    private DcMotor viperSlideMotorTwo;
   // private DcMotor armMotor;
    private double ticksPerRotationOfLeft;
    private double ticksPerRotationOfRight;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    private static final int TARGET_POSITION_VS = 1000;
    private static final double VS_SPEED = 0.2;
    //private static final double ARM_SPEED = 0.3;
    //camera
    private Camera camera;
    private static final boolean USE_WEBCAM = true;

    //The variable to store our instance of the AprilTag processor.
    private AprilTagProcessor aprilTag;

    //The variable to store our instance of the vision portal.
    private VisionPortal myVisionPortal;

    private Servo grabberServo = null;
    private Servo armServo = null;
    private Servo armServoTwo = null;

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public static final double GRABBER_SPEED = 0.10;
    public static final double GRABBER_MIN = 0.10 ;
    public static final double GRABBER_MAX = 0.40;
    private static double grabberDrive = 0.0;

    public static final double ARM_SPEED = 0.01;
    public static final double ARM_MIN = 0.10 ;
    public static final double ARM_MAX = 0.60;
    private static double armDrive = 0.3;

    //public static final double ARM_MIN_TWO = 0.05 ;
    //public static final double ARM_MAX_TWO = 0.90;
    public static final double ARM_MIDDLE = 0;

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        //Mapping hardware devices
        rfMotor = myOpMode.hardwareMap.get(DcMotor.class, "motorRF");
        rbMotor = myOpMode.hardwareMap.get(DcMotor.class, "motorRB");
        lfMotor = myOpMode.hardwareMap.get(DcMotor.class, "motorLF");
        lbMotor = myOpMode.hardwareMap.get(DcMotor.class, "motorLB");

        //setViperSlideMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // armMotor = myOpMode.hardwareMap.get(DcMotor.class,"motorArm");

        viperSlideMotor = myOpMode.hardwareMap.get(DcMotor.class, "motorvs");
        viperSlideMotorTwo = myOpMode.hardwareMap.get(DcMotor.class,"motorvstwo");
        grabberServo = myOpMode.hardwareMap.get(Servo.class, "gripperservo");
        armServo = myOpMode.hardwareMap.get(Servo.class, "armServo");
        armServoTwo = myOpMode.hardwareMap.get(Servo.class, "armServoTwo");
        armServoTwo.setDirection(Servo.Direction.REVERSE);
        //armServo.setPosition(0);
        //armServoTwo.setPosition(0);
        //armServo.resetDeviceConfigurationForOpMode();
        //armServoTwo.resetDeviceConfigurationForOpMode();
        setViperSlideMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        myOpMode.telemetry.addData("Status", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Initialize all the robot's camera.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void initCamera() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        /*myVisionPortal = new VisionPortal.Builder()
                    .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "webcam1"))
                    .addProcessors( aprilTag)
                    .build();
*/
    }   // end initDoubleVision()

    public AprilTagProcessor getAprilTag(){
        return aprilTag;
    }

    public void setMotorsMode(DcMotor.RunMode MotorRunMode) {
        rfMotor.setMode(MotorRunMode);
        rbMotor.setMode(MotorRunMode);
        lfMotor.setMode(MotorRunMode);
        lbMotor.setMode(MotorRunMode);

    }

    public int getLeftMotorCurrentPosition(){
        return lfMotor.getCurrentPosition() ;
    }
    public int getRightMotorCurrentPosition(){
        return rfMotor.getCurrentPosition();
    }
    public double getLeftMotorRotations(){
        return lfMotor.getCurrentPosition() / ticksPerRotationOfLeft;
    }
    public double getRightMotorRotations(){
        return rfMotor.getCurrentPosition() / ticksPerRotationOfRight;
    }

    public void setLeftTargetPosition(int leftTarget){
        lfMotor.setTargetPosition(leftTarget);
        lbMotor.setTargetPosition(leftTarget);
    }

    public void setRightTargetPosition(int rightTarget){
        rfMotor.setTargetPosition(rightTarget);
        rbMotor.setTargetPosition(rightTarget);
    }

    public boolean isLeftMotorBusy()
    {
        return lfMotor.isBusy();
    }

    public boolean isRightMotorBusy()
    {
        return rfMotor.isBusy();
    }

    public void setMotorDirection(MotorDirection direction){
        if (direction == MotorDirection.BACKWARD)
        {
            lfMotor.setDirection(DcMotor.Direction.FORWARD);
            lbMotor.setDirection(DcMotor.Direction.FORWARD);
            rfMotor.setDirection(DcMotor.Direction.REVERSE);
            rbMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        else
        {
            lfMotor.setDirection(DcMotor.Direction.REVERSE);
            lbMotor.setDirection(DcMotor.Direction.REVERSE);
            rfMotor.setDirection(DcMotor.Direction.FORWARD);
            rbMotor.setDirection(DcMotor.Direction.FORWARD);
        }
    }
    public void setViperSlideDirectionForward(){
        viperSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        viperSlideMotorTwo.setDirection(DcMotor.Direction.REVERSE);}
    public void setViperSlideDirectionReverse(){
        viperSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        viperSlideMotorTwo.setDirection(DcMotor.Direction.FORWARD);}
    public void reverseMotors(){
        setMotorDirection(MotorDirection.BACKWARD);
    }


    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double Drive, double Turn) {
        // Combine drive and turn for blended motion.
        double left  = Drive + Turn;
        double right = Drive - Turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(left, right);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        //leftDrive.setPower(leftWheel);
        //rightDrive.setPower(rightWheel);

        rfMotor.setPower(rightWheel);
        rbMotor.setPower(rightWheel);
        lfMotor.setPower(leftWheel);
        lbMotor.setPower(leftWheel);
    }

    public void goDiagonal(double num){
        //rfMotor.setPower(-num);
        rbMotor.setPower(num);
        lfMotor.setPower(num);
        //lbMotor.setPower(-num);
    }

    public void goStrafe (double num){
        rbMotor.setPower(-num);
        lbMotor.setPower(-num);
        rfMotor.setPower(num);
        lfMotor.setPower(num);
    }

    public void setViperSlideMotorMode (DcMotor.RunMode mode){
        viperSlideMotor.setMode(mode);
        viperSlideMotorTwo.setMode(mode);
    }

    public void setViperSlideMotorTargetPosition (){
        viperSlideMotor.setTargetPosition(TARGET_POSITION_VS);
        viperSlideMotorTwo.setTargetPosition(TARGET_POSITION_VS);
    }


    public void setViperSlideMotorPower (double power){
        viperSlideMotor.setPower(power);
        viperSlideMotorTwo.setPower(power);

    }

    public void moveGrabber(boolean closeGrabber){
        if (closeGrabber && grabberDrive > GRABBER_MIN) grabberDrive -= GRABBER_SPEED;

        if (!closeGrabber && grabberDrive < GRABBER_MAX) grabberDrive += GRABBER_SPEED;

        grabberServo.setPosition(Range.clip(grabberDrive, GRABBER_MIN, GRABBER_MAX));
    }

    public void moveGrabberToPosition(double position){
        grabberServo.setPosition(Range.clip(position, GRABBER_MIN, GRABBER_MAX));
    }

    public double getArmServoPosition(){
        return armServo.getPosition();
    }
    public double getArmServoTwoPosition(){
        return armServoTwo.getPosition();
    }

    public void moveArm(boolean closeArm){
        if ( closeArm && armDrive > ARM_MIN){
            armDrive -= ARM_SPEED;
            myOpMode.telemetry.addData("Arm", "back");
        }
        if ( !closeArm && armDrive < ARM_MAX){
            armDrive += ARM_SPEED;
            myOpMode.telemetry.addData("Arm", "forward");
        }

        myOpMode.telemetry.addData("Arm Position: ", Range.clip(armDrive, ARM_MIN, ARM_MAX));
        //armServo.setPosition(Range.clip(armDrive, ARM_MIN, ARM_MAX));
        //armServoTwo.setPosition(Range.clip(armDrive, ARM_MIN_TWO, ARM_MAX_TWO));
        moveArmToPosition(armDrive);
    }




    public void moveArmToPosition(double position){
        armServo.setPosition(Range.clip(position, ARM_MIN, ARM_MAX));
        armServoTwo.setPosition(Range.clip(position, ARM_MIN, ARM_MAX));
        //armServo.setPosition((int)position);
    }


    // All  of the arm functions for manual below
   /* public void setArmPower (){
        armMotor.setPower(VS_SPEED);
    }
    public void setArmMotorPowerZero (){
armMotor.setPower(0);
    }
    public void setArmDirectionForward(){
        armMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    public void setArmDirectionReverse(){
        armMotor.setDirection(DcMotor.Direction.REVERSE);
    }
*/
    /*
     * Pass the requested arm power to the appropriate hardware drive motor
     *      Power is reduced by 25%
     *
     * @param power Should the Arm be moved up or down
     */
}


