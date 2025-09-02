/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class ManualManager {

    public static enum MotorDirection {
        FORWARD,
        BACKWARD
    }
    private Blinker control_Hub;
    private IMU imu;

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // naming motors
    private HornetRobo hornetRobo;
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


    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */

    // Define a constructor that allows the OpMode to pass a reference to itself.

    public static final double GRABBER_SPEED = 0.10;
    public static final double GRABBER_MIN = 0.10 ;
    public static final double GRABBER_MAX = 0.40;
    private static double grabberDrive = 0.0;

    public static final double ARM_SPEED = 0.01;
    public static final double ARM_MIN = 0.10 ;
    public static final double ARM_MAX = 0.90;
    private static double armDrive = 0.3;

    //public static final double ARM_MIN_TWO = 0.05 ;
    //public static final double ARM_MAX_TWO = 0.90;
    public static final double ARM_MIDDLE = 0;

    private ArmManager autoArmManager;

    private ViperSlideManager autoViperSlideManager;

    private GrabberManager autoGrabberManager;

    private DriveManager autoDriveManager;


    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public ManualManager(LinearOpMode opmode, HornetRobo hornetRoboObj)  {
        myOpMode = opmode;

        hornetRobo = hornetRoboObj;

        autoArmManager = new ArmManager(opmode, hornetRobo);

        autoDriveManager = new DriveManager(opmode, hornetRobo);

        autoGrabberManager = new GrabberManager(opmode, hornetRobo);

        autoViperSlideManager = new ViperSlideManager(opmode, hornetRobo);
    }
    public void Init()    {
        //Mapping hardware devices
        HardwareMapper.MapToHardware(myOpMode, hornetRobo);
        autoArmManager.SetDirection(DriveManager.DriveDirection.FORWARD);
        //armServo.setPosition(0);
        //armServoTwo.setPosition(0);
        //armServo.resetDeviceConfigurationForOpMode();
        //armServoTwo.resetDeviceConfigurationForOpMode();
        autoViperSlideManager.SetMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        myOpMode.telemetry.addData("Status", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Initialize all the robot's camera.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void InitCamera() {
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
                    .build();*/

    }   // end initDoubleVision()

    public AprilTagProcessor GetAprilTag(){
        return aprilTag;
    }

    public void SetMotorsMode(DcMotor.RunMode MotorRunMode) {

        autoDriveManager.SetAllMotorsMode(MotorRunMode);
    }


    public double GetLeftMotorRotations(){
        return autoDriveManager.GetCurrentPosition(DriveManager.MotorPosition.LEFT_FRONT) /ticksPerRotationOfLeft;
    }
    public double GetRightMotorRotations(){
        return autoDriveManager.GetCurrentPosition(DriveManager.MotorPosition.RIGHT_FRONT) / ticksPerRotationOfRight;
    }

    public void SetLeftTargetPosition(int leftTargetPosition){
        autoDriveManager.SetTargetPosition(DriveManager.DriveDirection.LEFT, leftTargetPosition);

    }

    public void SetRightTargetPosition(int rightTargetPosition){
        autoDriveManager.SetTargetPosition(DriveManager.DriveDirection.RIGHT, rightTargetPosition);
    }

    public boolean IsLeftMotorBusy()
    {
        return autoDriveManager.IsMotorBusy(DriveManager.MotorPosition.LEFT_FRONT);
    }

    public boolean isRightMotorBusy()
    {
        return autoDriveManager.IsMotorBusy(DriveManager.MotorPosition.RIGHT_FRONT);
    }

    public void SetMotorDirection(DriveManager.DriveDirection Direction){
       autoDriveManager.SetMotorDirection(Direction);
    }

    public void SetViperSlideDirectionForward() {
        autoViperSlideManager.SetDirection(DriveManager.DriveDirection.FORWARD);
    }

    public void SetViperSlideDirectionReverse() {
        autoViperSlideManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
    }

    public void ReverseMotors(){
        autoDriveManager.SetMotorDirection(DriveManager.DriveDirection.BACKWARD);
    }


    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void DriveRobot(double Drive, double Turn) {
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
        autoDriveManager.SetDrivePower(left, right);

    }

    public void GoDiagonal(double Num){
        autoDriveManager.SetPowerToGoDiagonal(Num);
    }

    public void GoStrafe (double Num){
        autoDriveManager.SetPowerToStrafe(Num);
    }

    public void SetViperSlideMotorMode (DcMotor.RunMode Mode){
       autoViperSlideManager.SetMotorsMode(Mode);
    }

    public void SetViperSlideMotorTargetPosition (){
        autoViperSlideManager.SetTargetPosition(TARGET_POSITION_VS);
    }


    public void SetViperSlideMotorPower (double Power){
        autoViperSlideManager.SetPower(Power);

    }

    public void MoveGrabber(boolean closeGrabber){
        if (closeGrabber && grabberDrive > GRABBER_MIN) grabberDrive -= GRABBER_SPEED;

        if (!closeGrabber && grabberDrive < GRABBER_MAX) grabberDrive += GRABBER_SPEED;

        autoGrabberManager.SetGrabberToPosition(Range.clip(grabberDrive, GRABBER_MIN, GRABBER_MAX));
    }

    public void MoveGrabberToPosition(double Position){
        autoGrabberManager.SetGrabberToPosition(Position);
    }

    public void MoveArm(boolean CloseArm){
        if ( CloseArm && armDrive > ARM_MIN){
            armDrive -= ARM_SPEED;
            myOpMode.telemetry.addData("Arm", "back");
        }
        if ( !CloseArm && armDrive < ARM_MAX){
            armDrive += ARM_SPEED;
            myOpMode.telemetry.addData("Arm", "forward");
        }

        myOpMode.telemetry.addData("Arm Position: ", Range.clip(armDrive, ARM_MIN, ARM_MAX));
        //armServo.setPosition(Range.clip(armDrive, ARM_MIN, ARM_MAX));
        //armServoTwo.setPosition(Range.clip(armDrive, ARM_MIN_TWO, ARM_MAX_TWO));
        MoveArmToPosition(armDrive);
    }

    public void MoveArmToPosition(double Position){
        autoArmManager.MoveArmToPosition(Position, ARM_MIN, ARM_MAX);

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


