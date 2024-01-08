package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.abs;

import android.util.Size;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This class contains methods to control drive base movement
 */
public class Movement {

    public int motor_ticks;
    public static enum Direction {
        FRONT,
        BACK,
        LEFT,
        RIGHT
    }
    private DcMotor lfDrive;
    private DcMotor rfDrive;
    private DcMotor lbDrive;
    private DcMotor rbDrive;
    private IMU imu;

    private Telemetry telemetry;

    //AprilTag variables:
    // Used for managing the AprilTag detection process.
    private AprilTagProcessor myAprilTagProcessor;

    // Used to manage the video source.
    private VisionPortal myVisionPortal;

    private HardwareMap map;

    int alignStage = 0;
    double currentX = -2;
    double currentY = 15;

    boolean tagDetected = false;
    boolean aprilTagAligned = false;
    double axial = 0;
    double lateral = 0;
    double yaw = 0;
    double currentAngle = 0;
    // <<< end apriltag variables

    private RevBlinkinLedDriver blinkinLED;

    // Tracks if it's quicker to turn right or left
    double turnError = 0;

    double moveStartDirection = 0.0;

    /**
     * Pulls in information about the motors that is determined during initialization and makes
     * that information accessible to the rest of the class.
     * @param leftFrontDrive  the front left wheels motor,
     * @param  rightFrontDrive  the front right wheels motor,
     * @param  leftBackDrive  the back left wheels motor,
     * @param  rightBackDrive  the back right wheels motor
     */
    public Movement(DcMotor leftFrontDrive, DcMotor rightFrontDrive,
                    DcMotor leftBackDrive, DcMotor rightBackDrive, IMU imu1, HardwareMap hMap, Telemetry telemetry1){
        lfDrive = leftFrontDrive;
        rfDrive = rightFrontDrive;
        lbDrive = leftBackDrive;
        rbDrive = rightBackDrive;
        map = hMap;
        imu = imu1;
        telemetry = telemetry1;
        blinkinLED = map.get(RevBlinkinLedDriver.class, "blinkin");

        // Build the AprilTag processor
        // set parameters of AprilTagProcessor, then use Builder to build
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                //.setTagLibrary(myAprilTagLibrary)
                //.setNumThreads(tbd)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        // set apriltag resolution decimation factor
        myAprilTagProcessor.setDecimation(2);


        // Build the vision portal
        // set parameters,then use vision builder.
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(map.get(WebcamName.class, "gge_backup_cam"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                //.setCameraResolution(new Size(1280,720))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }


    public void initIMU () {
        // Initialize the IMU.
        // For Tiny - Logo UP; USB BACKWARD
        // For Rosie - Logo BACKWARD; USB UP
        // For Gge - Logo UP; USB FORWARD;
        // Initializes the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction that the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    /**
     * Resets all wheel motor encoder positions to 0
     */
    private void initMovement(boolean resetEncoders){
        motor_ticks = 0;

        moveStartDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (resetEncoders) {
            lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
            //lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
            rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
            //rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
            lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
            //lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
            rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
            //rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        }
        // Set default to BRAKE mode for control
        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Moves all wheel motors forward a distance in ticks
     * @param power  the power given to the motors
     */
    public void Forward(int ticks, double power){
        initMovement(true);
        lfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while(abs (lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30){
            if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > moveStartDirection){
                lfDrive.setPower(power * 1.15);
                lbDrive.setPower(power * 1.15);
            } else {
                lfDrive.setPower(power * 0.85);
                lbDrive.setPower(power * 0.85);
            }
        }
    }

    /**
     * Moves all wheel motors backwards a distance in ticks
     * @param power  the power given to the motors
     */
    public void Backwards(int ticks, double power){
        initMovement(true);
        lfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while(abs (lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30){
            if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > moveStartDirection){
                lfDrive.setPower(power * 0.85);
                lbDrive.setPower(power * 0.85);
            } else {
                lfDrive.setPower(power * 1.15);
                lbDrive.setPower(power * 1.15);
            }
        }
    }

    /**
     * Moves the front right and the back left motor forwards, and the front left and the back right
     * motors backwards in order to move left a distance in ticks
     * 
     * @param power  the power given to the motors
     */
    public void Left(int ticks, double power){
        initMovement(true);
        lfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while(abs (lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30){
            if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > moveStartDirection){
                lbDrive.setPower(power * 1.15);
                rbDrive.setPower(power * 1.15);
            } else {
                lbDrive.setPower(power * 0.85);
                rbDrive.setPower(power * 0.85);
            }
        }
    }

    /**
     * Moves the front left and the back right motor forwards, and the front right and the back left
     * motors backwards in order to move left a distance in ticks
     * @param power  the power given to the motors
     */
    public void Right(int ticks, double power){
        initMovement(true);
        lfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while(abs (lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30){
            if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > moveStartDirection){
                lbDrive.setPower(power * 0.85);
                rbDrive.setPower(power * 0.85);
            } else {
                lbDrive.setPower(power * 1.15);
                rbDrive.setPower(power * 1.15);
            }
        }
    }

    /**
     * turns the robot in place a distance in degrees
     * @param degrees - the distance of the rotation in degrees
     */
    public void Rotate(double degrees){

        double currentDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        turnError = degrees + 2.5; // This works starting with +2.5 degrees (possibly inertia)

        // Through measurement, Gge takes 1500 ticks to complete a full rotation
        // Set the initial stepSize to what we expect the rotation to need
        int stepSize = (int) (turnError / 360 * 1580);

        // Move slowly and in increments
        //Move the right front motor forward
        rfDrive.setTargetPosition(rfDrive.getCurrentPosition() + (stepSize * -1));
        // Move the left front motor backwards
        lfDrive.setTargetPosition(lfDrive.getCurrentPosition() + (stepSize));
        // Move the right back motor forward
        rbDrive.setTargetPosition(rbDrive.getCurrentPosition() + (stepSize * -1));
        // Move the left back motor backward
        lbDrive.setTargetPosition(lbDrive.getCurrentPosition() + (stepSize));

        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rfDrive.setPower(0.5);
        lfDrive.setPower(0.5);
        rbDrive.setPower(0.5);
        lbDrive.setPower(0.5);

        currentDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        turnError = degrees - currentDirection;
        //Closed loop turn.  Stay in the while loop until the desired bering is achieved.
        while (abs (lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) < 10) {
        }
    }

    /**
     * Calculates the turn error and contains it within 180 and -180
     * @param desiredDirection - which direction you want to go to
     * @param currentDirection - your current direction
     */
    public static double CalcTurnError(double desiredDirection, double currentDirection){
        double turnDiff = desiredDirection - currentDirection;
        if (turnDiff < -180) {
            turnDiff = turnDiff + 360;
        } else if (turnDiff > 180) {
            turnDiff = turnDiff - 360;
        }
        return turnDiff;
    }

    public boolean GoToAprilTag(int tagNumber) {

        initMovement(false);

        if (alignStage == 0) {
            // Set Powers to 0 for safety and not knowing what they are set to.
            StopMotors();

            // Init variables for April motion
            axial = 0;
            lateral = 0;
            yaw = 0;

            alignStage = 1;
        }

        double targetX = 0;
        // The AprilTag is not centered on the LEFT and RIGHT backdrop zones, adjust X targets
        if (tagNumber == 1 || tagNumber == 4) {
            targetX = 0.5;
        } else if (tagNumber == 3 || tagNumber == 6) {
            targetX = -0.5;
        }
        double targetY = 10;
        double targetAngle = 0;

        // Translate the tagNumber requested to know the angle of the backdrop in robot IMU
        if (tagNumber <= 3) {
            targetAngle = -90;
        } else if (tagNumber > 3) {
            targetAngle = 90;
        }

        currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Scan for April Tag detections and update current values if you find one.
        List<AprilTagDetection> tag = myAprilTagProcessor.getDetections();
        for (int i = 0; i < tag.size(); i++) {
            if (tag.get(i).id == tagNumber) {
                currentX = tag.get(i).ftcPose.x;
                currentY = tag.get(i).ftcPose.y;
                //blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                tagDetected = true;
            }
        }

        // Update Telemetry with key data
        telemetry.addData("tags found: ", tag.size());
        telemetry.addData("AlignStage: ", alignStage);
        telemetry.addData("Current X: ", currentX);
        telemetry.addData("Target X: ", targetX);
        telemetry.addData("Current Y: ", currentY);
        telemetry.addData("Target Y: ", targetY);
        telemetry.addData("Current Angle: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Target Angle: ", targetAngle);
        telemetry.update();

        // Like the driver control TeleOp, consider the needed axial, lateral and yaw for
        // the motion needed to get to the April Tag.
        // axial from drive is gamepad1.left_stick_y;
        // lateral from drive is -gamepad1.left_stick_x;
        // yaw from drive is -gamepad1.right_stick_x;



        if (!tagDetected){
            axial = -0.15;
            aprilTagAligned = false;
        }

        // Square up the robot to the backdrop (from targetAngle above)
        // If the yaw is +, apply -yaw, if the yaw if -, apply +yaw (-right_stick_x in robot mode)
        if (abs (targetAngle - currentAngle) > 2) {
            yaw = -Movement.CalcTurnError(targetAngle, currentAngle) / 45;
            if (yaw > 0.3){
                yaw = 0.3;
            } else if (yaw < -0.3){
                yaw = -0.3;
            }
        } else {
            yaw = 0;
        }

        // Slide laterally to correct for X or right motion
        // If the x distance is > 1 inch off of targetX move left or right accordingly
        // To make the robot go right, reduce the lateral (-left_stick_x in robot mode)
        // To make the robot go left, increase the lateral (-left_stick_x in robot mode)
        if (targetX - currentX > 1) {
            lateral = 0.2;
        } else if (targetX - currentX < -1) {
            lateral = -0.2;
        }else {
            lateral = 0;
        }

        // Back the robot up to the right distance to raise the lift
        if (currentY > targetY) {
            axial = -0.15;
        } else {
            axial = 0;
        }

        // Combine the axial, lateral and yaw factors to be powers
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Apply calculated values to drive motors
        lfDrive.setPower(leftFrontPower);
        rfDrive.setPower(rightFrontPower);
        lbDrive.setPower(leftBackPower);
        rbDrive.setPower(rightBackPower);

        // Test to see if we are at all three parts of our desired position and we are aligned.
        if (abs (targetX - currentX) < 1 && currentY < targetY && abs (targetAngle - currentAngle) < 2){
            aprilTagAligned = true;
            //blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        }
        return aprilTagAligned;
    }


    public void StopMotors() {
        // Set Powers to 0 for safety and not knowing what they are set to.
        lfDrive.setPower(0);
        rfDrive.setPower(0);
        lbDrive.setPower(0);
        rbDrive.setPower(0);
    }
}
