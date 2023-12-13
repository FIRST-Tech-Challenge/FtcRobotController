package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pipeline.GripPipelineWhitePixelRGBT1;
import org.firstinspires.ftc.teamcode.utility.GamePieceLocation;
import org.firstinspires.ftc.teamcode.utility.GamepiecePositionFinder;
import org.firstinspires.ftc.teamcode.utility.IntakeMovement;
import org.firstinspires.ftc.teamcode.utility.LinearSlideMovement;
import org.firstinspires.ftc.teamcode.utility.Movement;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class AutoBase extends OpMode {
    static final int LOW_LINEAR_SLIDE_TICKS = 200; // Low position for the linear slides
    static final int BOTTOM_LINEAR_SLIDE_TICKS = 0; // Bottom position for the linear slides
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 960; // modify for your camera
    // Declare OpMode members for each of the 4 motors.
    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;
    protected IMU imu;
    protected GamePieceLocation gamepieceLocation;
    Servo leftClaw;
    Servo rightClaw;
    Servo wrist;
    Servo conveyor;
    OpenCvWebcam webcam;
    GripPipelineWhitePixelRGBT1 pipeline;
    Movement moveTo;
    IntakeMovement intake;
    LinearSlideMovement linearSlideMove;
    int state;
    double rightCount = 0;
    double centerCount = 0;
    double leftCount = 0;
    private DcMotor leftLinearSlide = null;
    private DcMotor rightLinearSlide = null;

    // Motor is 28 ticks per revolution
    // Gear Ratio is 12:1
    // Wheel diameter is 100mm
    double ticksPerInch = (28 * 12) / ((100 * 3.14) / 25.4);



    protected void estimateLocation(GamepiecePositionFinder gamePiecePOS) {
        Point avgLoc = pipeline.avgContourCoord();
        if (gamePiecePOS.getPOS() == GamePieceLocation.RIGHT) {
            rightCount += 1;
        } else if (gamePiecePOS.getPOS() == GamePieceLocation.CENTER) {
            centerCount += 1;
        } else if (gamePiecePOS.getPOS() == GamePieceLocation.LEFT) {
            leftCount += 1;
        }

        if (rightCount > centerCount && rightCount > 5) {
            gamepieceLocation = GamePieceLocation.RIGHT;
        } else if (centerCount > leftCount && centerCount > 5) {
            gamepieceLocation = GamePieceLocation.CENTER;
        } else if (leftCount > 5) {
            gamepieceLocation = GamePieceLocation.LEFT;
        }

        // Reset the counters to lower values every 50 detects to allow for field condition changes
        if (rightCount + centerCount + leftCount > 50) {
            rightCount = rightCount * 0.3;
            centerCount = centerCount * 0.3;
            leftCount = leftCount * 0.3;
        }

        telemetry.addData("AvgContour.x", avgLoc.x);
        telemetry.addData("AvgContour.y", avgLoc.y);
        telemetry.addData("Left Probability", leftCount / (leftCount + rightCount + centerCount));
        telemetry.addData("Center Probability", centerCount / (leftCount + rightCount + centerCount));
        telemetry.addData("Right Probability", rightCount / (leftCount + rightCount + centerCount));
        telemetry.addData("location", gamepieceLocation);
        telemetry.addData("state", state);
        telemetry.update();
    }

    @Override
    public void init() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "gge_cam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new GripPipelineWhitePixelRGBT1();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed", "");
                telemetry.update();
            }
        });

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        conveyor = hardwareMap.get(Servo.class, "conveyor");

        imu = hardwareMap.get(IMU.class, "imu");

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
        ((DcMotorEx) leftFrontDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        ((DcMotorEx) leftBackDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        ((DcMotorEx) rightFrontDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        ((DcMotorEx) rightBackDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        // For Lift, PIDF values set to reduce jitter on high lift
        ((DcMotorEx) leftLinearSlide).setVelocityPIDFCoefficients(8, 0.75, 0, 4);
        ((DcMotorEx) rightLinearSlide).setVelocityPIDFCoefficients(8, 0.75, 0, 4);

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

        leftLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = new IntakeMovement(rightClaw, leftClaw, wrist, telemetry);
        moveTo = new Movement(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, imu, telemetry);
        linearSlideMove = new LinearSlideMovement(leftLinearSlide, rightLinearSlide, intake);

        state = 0;
        //drive speed limiter
        //double powerFactor = 0.25;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        runtime.reset();
    }
    protected void displayTelemetry(double DirectionNow) {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Direction Now", JavaUtil.formatNumber(DirectionNow, 2));
        telemetry.addData("Target Position", leftFrontDrive.getTargetPosition());
        telemetry.addData("Left Front Pos", leftFrontDrive.getCurrentPosition());
        telemetry.addData("Right Front Pos", rightFrontDrive.getCurrentPosition());
        telemetry.addData("Left Back Pos", leftBackDrive.getCurrentPosition());
        telemetry.addData("Right Back Pos", rightBackDrive.getCurrentPosition());
        telemetry.addData("state", state);
        telemetry.addData("location", gamepieceLocation);
        telemetry.update();
    }
}
