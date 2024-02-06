package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.AllianceColour;
import org.firstinspires.ftc.teamcode.utility.AprilTagLocation;
import org.firstinspires.ftc.teamcode.utility.VisionProcessorMode;
import org.firstinspires.ftc.teamcode.utility.VisionSystem;
import org.firstinspires.ftc.teamcode.utility.GamePieceLocation;
import org.firstinspires.ftc.teamcode.utility.IntakeMovement;
import org.firstinspires.ftc.teamcode.utility.LinearSlideMovement;
import org.firstinspires.ftc.teamcode.utility.Movement;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;

import java.util.List;

public abstract class AutoBase extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime(); //
    protected DcMotorEx leftFrontDrive = null;
    protected DcMotorEx leftBackDrive = null;
    protected DcMotorEx rightFrontDrive = null;
    protected DcMotorEx rightBackDrive = null;
    protected IMU imu;
    MecanumDriveKinematics kinematics;
    MecanumDriveOdometry odometry;
    ElapsedTime odometryTimer = new ElapsedTime();
    MecanumDriveWheelSpeeds odometrySpeeds = new MecanumDriveWheelSpeeds();
    private RevBlinkinLedDriver blinkinLED;
    protected GamePieceLocation gamepieceLocation;
    Servo leftClaw;
    Servo rightClaw;
    Servo conveyor;
    Movement moveTo;
    IntakeMovement intake;
    LinearSlideMovement linearSlideMove;
    int state;
    private DcMotorEx leftLinearSlide = null;
    private DcMotorEx rightLinearSlide = null;
    private DcMotorEx wrist = null;




    // Motor is 28 ticks per revolution
    // Gear Ratio is 12:1
    // Wheel diameter is 100mm
    final static double ticksPerInch = (28 * 12) / ((100 * 3.14) / 25.4);
    protected VisionSystem visionSystem;
    protected static Pose2d lastFieldPos;



    protected static AllianceColour allianceColour;

    protected double DirectionNow;
    protected static List<AprilTagLocation> targetAprilTags;

    @Override
    public void runOpMode() {

        //we can pass in the hardwareMap now that we are in runOpMode
        // this is a well documented technique
        visionSystem = new VisionSystem(hardwareMap, telemetry);

        // wait for the cameras to start streaming before we proceed
        while(opModeInInit()){
            if(visionSystem.camerasReady()){
                break;
            };
            telemetry.addData("camera state:",visionSystem.camerasReady());
            telemetry.update();
        }
        // setting which Vision Processing mode we want here.
        VisionProcessorMode currentVPMode = visionSystem.setVisionProcessingMode(VisionProcessorMode.FRONT_CAMERA_GAMEPIECE);

        if(opModeInInit()) {
            // After INIT has been pressed we will run this part of the code.  During this phase
            // we want to be able to see a preview of the what the HSVProcessor is seeing
            visionSystem.resumeLiveView();
        }
        telemetry.addData("Current VPMode",currentVPMode);
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        leftLinearSlide = hardwareMap.get(DcMotorEx.class, "left_linear_slide");
        rightLinearSlide = hardwareMap.get(DcMotorEx.class, "right_linear_slide");
        wrist = hardwareMap.get(DcMotorEx.class, "wrist");

        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        conveyor = hardwareMap.get(Servo.class, "conveyor");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        blinkinLED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        double DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Adding in PIDF Config values learned from previous testing
        // These may need to be tuned anytime the motor weights or config changes.
        // Set PIDF values thinking of ...
        // ...P as primary force (set second)
        // ...I as smoothing (set last)
        // ...D as deceleration (set third)
        // ...F as holding / static force (set first)
        // For Mecanum drive, 8, 0, 0.5, 5 works well on Tiny
        // ... and 7, 0.2, 0.1, 8 works on Rosie (heavier bot)
        leftFrontDrive.setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        leftBackDrive.setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        rightFrontDrive.setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        rightBackDrive.setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        // For Lift, PIDF values set to reduce jitter on high lift
        leftLinearSlide.setVelocityPIDFCoefficients(8, 0.75, 0, 8);
        rightLinearSlide.setVelocityPIDFCoefficients(8, 0.75, 0, 8);
        // For Wrist, PIDF values set to reduce jitter
        wrist.setVelocityPIDFCoefficients(15, 0.2, 0.05, 16);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Somehow this is reversed from the TeleOp Gge program.
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        wrist.setDirection(DcMotor.Direction.REVERSE);

        intake = new IntakeMovement(rightClaw, leftClaw, wrist, conveyor, telemetry);
        // Pass all needed hardware to the Movement class including the AprilTag detection for refactored GoToAprilTag
        moveTo = new Movement(leftFrontDrive,
                            rightFrontDrive,
                            leftBackDrive,
                            rightBackDrive,
                            imu,
                            blinkinLED,
                            odometry,
                            kinematics,
                            odometryTimer,
                            odometrySpeeds,
                            telemetry,
                            visionSystem);
        linearSlideMove = new LinearSlideMovement(leftLinearSlide, rightLinearSlide, intake);

        // getting the initialized odometry object
        odometry = moveTo.getOdometry();
        state = 0;
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    protected void displayTelemetry(double DirectionNow) {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Direction Now", JavaUtil.formatNumber(DirectionNow, 2));
        telemetry.addData("GamePiece Location: ", gamepieceLocation);
        telemetry.addData("Auto State Now: ", state);
        telemetry.addData("Odometry (X, Y, Angle)", "%4.2f, %4.2f, %4.2f",
                odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(),
                odometry.getPoseMeters().getRotation().getDegrees());
        telemetry.addData("*******updating pos********",lastFieldPos);
        telemetry.update();
    }

    protected void updateOdometry() {
        DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        odometrySpeeds = moveTo.GetWheelSpeeds();
        odometry.updateWithTime(odometryTimer.seconds(),
                new Rotation2d(Math.toRadians(DirectionNow)), odometrySpeeds);
    }

    protected void updateLastPos(){
        lastFieldPos = moveTo.getOdometry().getPoseMeters();
    }

    protected void setFieldPosition(FieldPosition fPos) {
       visionSystem.setFieldPosition(fPos);
    }

    protected SpikePosition getSpikePosition() {
        return visionSystem.getSpikePosition();
    }

    protected double getLeftSpikeSaturation() {
        return visionSystem.getLeftSpikeSaturation();
    }

    protected double getCenterSpikeSaturation() {
        return visionSystem.getCenterSpikeSaturation();
    }

    protected double getRightSpikeSaturation() {
        return visionSystem.getRightSpikeSaturation();
    }

    protected void setInitFieldPos(Pose2d initPos, List<AprilTagLocation> aTags){
        lastFieldPos = initPos;
        targetAprilTags = aTags;
    }

    public static Pose2d getLastFieldPos(){
        return lastFieldPos;
    }

    public static AllianceColour getAllianceColour() {
        return allianceColour;
    }

}

