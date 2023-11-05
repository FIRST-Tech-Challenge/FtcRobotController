/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

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

    private Blinker control_Hub;
    private IMU imu;

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // naming motors
    private DcMotor rfMotor;
    private DcMotor rbMotor;
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private DcMotor armMotorL   = null;
    private DcMotor armMotorR   = null;
    private DcMotor leftHand    = null;
    private DcMotor rightHand   = null;

    private Servo   elbowServo  = null;
    private Servo   grabberServo= null;
    private Servo   launcher    = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    public static final double LAUNCHER_MIN     = 0.0;
    public static final double LAUNCHER_MAX     = 0.4;
    public static final double ELBOW_MIN        = 0.05;
    public static final double ELBOW_MAX        = 0.80;
    public static final double GRABBER_MIN        = 0.25;
    public static final double GRABBER_MAX        = 0.60;

    private static double elbowDrive           = 0.5;
    private static double grabberDrive         = 0.0;

    //camera
    private Camera camera;
    private static final boolean USE_WEBCAM = true;

    //color sensor
    private NormalizedColorSensor leftColorSensor;
    private NormalizedColorSensor rightColorSensor;

    //distance sensor
    private DistanceSensor sensorDistance;

    //The variable to store our instance of the AprilTag processor.
    private AprilTagProcessor aprilTag;

    //The variable to store our instance of the TensorFlow Object Detection processor.
    private TfodProcessor tfod;

    //The variable to store our instance of the vision portal.
    private VisionPortal myVisionPortal;

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    private View relativeLayout;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

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

        lfMotor.setDirection(DcMotor.Direction.REVERSE);
        lbMotor.setDirection(DcMotor.Direction.REVERSE);

        //servos
        launcher = myOpMode.hardwareMap.get(Servo.class, "launcher");
        //launcher.setDirection(Servo.Direction.REVERSE);
        launcher.setPosition(LAUNCHER_MIN);

        //arm motors
        armMotorL       = myOpMode.hardwareMap.get(DcMotor.class, "motorlefthex");
        armMotorR       = myOpMode.hardwareMap.get(DcMotor.class, "motorrighthex");
        elbowServo      = myOpMode.hardwareMap.get(Servo.class, "elbow");
        grabberServo    = myOpMode.hardwareMap.get(Servo.class, "grabber");
        elbowServo.resetDeviceConfigurationForOpMode();
        grabberServo.resetDeviceConfigurationForOpMode();
        armMotorL.setDirection(DcMotor.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //leftHand = myOpMode.hardwareMap.get(DcMotor.class, "motorlefthex");
        //leftHand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        /* when ready to use the left/right color sensor
        leftColorSensor =
        .hardwareMap.get(NormalizedColorSensor.class, "color_left");
        rightColorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "color_right");
        */

        // you can use this as a regular DistanceSensor.
        sensorDistance = myOpMode.hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

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
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "webcam1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

    public AprilTagProcessor getAprilTag(){
        return aprilTag;
    }

    public void reverseMotors(){
        lfMotor.setDirection(DcMotor.Direction.FORWARD);
        lbMotor.setDirection(DcMotor.Direction.FORWARD);
        rfMotor.setDirection(DcMotor.Direction.REVERSE);
        rbMotor.setDirection(DcMotor.Direction.REVERSE);
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

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *      Power is reduced by 25%
     *
     * @param power Should the Arm be moved up or down
     */
    public void moveArm (double power) {
        armMotorL.setPower(power * 0.25);
        armMotorR.setPower(power * 0.25);
    }

    /**
     * Stop the arm from moving
     *
     */
    public void stopArm(){
        armMotorL.setPower(0);
        armMotorR.setPower(0);
    }

    public void moveElbow(boolean moveUp){
        if (moveUp && elbowDrive > ELBOW_MIN) elbowDrive -= .01;

        if (!moveUp && elbowDrive < ELBOW_MAX) elbowDrive += .01;

        elbowServo.setPosition(Range.clip(elbowDrive, ELBOW_MIN, ELBOW_MAX));
    }

    public void moveElbowToPosition(double elbowDrive){

        elbowServo.setPosition(Range.clip(elbowDrive, ELBOW_MIN, ELBOW_MAX));
    }

    public void moveGrabber(boolean closeGrabber){
        if (closeGrabber && grabberDrive > GRABBER_MIN) grabberDrive -= .01;

        if (!closeGrabber && grabberDrive < GRABBER_MAX) grabberDrive += .01;

        grabberServo.setPosition(Range.clip(grabberDrive, GRABBER_MIN, GRABBER_MAX));
    }

    public double getGrabberPoistion(){
        return grabberServo.getPosition();
    }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setHandPositions(int offset, double speed) {
        //leftHand.setTargetPosition(leftHand.getCurrentPosition() + offset);
        //leftHand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //leftHand.setPower(speed);

        //offset = Range.clip(offset, -0.5, 0.5);
        //leftHand.setPosition(MID_SERVO + offset);
        //rightHand.setPosition(MID_SERVO - offset);
    }

    public void setLauncher(double position){
        launcher.setPosition(position);
    }

    public void resetLaunch(){
        launcher.setPosition(LAUNCHER_MIN);
    }

    public double getDistanceFromObject(){
        return sensorDistance.getDistance(DistanceUnit.INCH);
    }
}