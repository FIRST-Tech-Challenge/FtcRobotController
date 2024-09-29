package org.firstinspires.ftc.robotcontroller.previous_year_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous
@Disabled
public class BlueCloseAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    private Servo wristMotor = null;
    private Servo clawMotor1 = null;
    private Servo clawMotor2 = null;
    private Servo slingshotRelease = null;
    private DcMotor hangMotor = null;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */

    private VisionPortal myVisionPortal;

    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        //motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //servoTest = hardwareMap.get(Servo.class, "servoTest");

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_Motor");
        wristMotor = hardwareMap.get(Servo.class, "wrist_Motor");
        clawMotor1 = hardwareMap.get(Servo.class, "claw_Motor1");
        clawMotor2 = hardwareMap.get(Servo.class, "claw_Motor2");
        slingshotRelease = hardwareMap.get(Servo.class, "slingshot_Release");
        hangMotor = hardwareMap.get(DcMotor.class, "hang_Motor");
        //camera camera = new camera(hardwareMap);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        wristMotor.setDirection(Servo.Direction.REVERSE);
        clawMotor1.setDirection(Servo.Direction.FORWARD);
        clawMotor2.setDirection(Servo.Direction.REVERSE);
        slingshotRelease.setDirection(Servo.Direction.FORWARD);
        hangMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        AprilTagProcessor myAprilTagProcessor;
        // Create the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        TfodProcessor myTfodProcessor;
        // Create the TensorFlow Object Detection processor and assign it to a variable.
        myTfodProcessor = TfodProcessor.easyCreateWithDefaults();

        VisionPortal myVisionPortal;

        // Create a VisionPortal, with the specified camera and AprilTag processor, and assign it to a variable.
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor, myTfodProcessor);

        // Enable or disable the AprilTag processor.
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);
         */


        //camera.switchToFirstPipeline();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /*
        while (!isStopRequested())  {
            initDoubleVision();

            if (opModeInInit()) {
                telemetry.addData("DS preview on/off","3 dots, Camera Stream");
                telemetry.addLine();
                telemetry.addLine("----------------------------------------");
            }

            // OpMode receives the information transmitted from the pipeline class
            // to the camera module class.
            //telemetry.addLine(camera.getPipeline1Output());
            telemetry.update();
        }
         */

        sleep(1000);
        setDrive(0.0, 0.0, 0.0, 0.0, 0.3, true,false, 0.0);

        sleep(1000);
        setDrive(0.0, 0.0, 0.0, 0.9, 0.6, true,false, 0.0);
        sleep(1450);
        setDrive(1.0, 0.0, 0.0, 0.0, 0.6, true,false, 0.0);
        sleep(835);
        setDrive(0.0, 0.0, 0.0, 0.5, 0.8, true,false, 0.0);
        sleep(300);
        setDrive(0.0, 0.0, 0.0, 0.0,  0.8, false,false, 0.0);
        sleep(1500);
        setDrive(0.0, 0.0, 0.0, -1.0, 1.0, false,false, 0.0);
        sleep(1700);
        setDrive(-1.0, 0.0, 0.0, 0.0, 1.0, false,false, 0.0);
        sleep(650);

        setDrive(0.0, -1.0, 0.0, 0.0, 1.0, false, false, 0.0);
        sleep(1900);
        setDrive(0.0, 0.0, 0.0, 0.0, 1.0, false, false, 0.0);
        //sleep(10000);

    }
    private void setDrive(double setAxial, double setLateral, double setYaw, double setArmPitch, double setWristPitch, boolean setClawOpen, boolean setPlaneLaunch, double setHangStrength){
        double max;
        double wristMotorPower = 0.0;
        double clawMotorPower = 0.0;
        double slingshotPosition = 0.3;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = setAxial + setLateral + setYaw;
        double rightFrontPower = setAxial - setLateral - setYaw;
        double leftBackPower   = setAxial - setLateral + setYaw;
        double rightBackPower  = setAxial + setLateral - setYaw;
        double armMotorPower = 0.70 * setArmPitch;
        double hangMotorPower = setHangStrength;
        wristMotorPower = setWristPitch;

        if (setClawOpen) {
            clawMotorPower = 0.0;
        }
        else {
            clawMotorPower = 1.0;
        }

        if (setPlaneLaunch){
            slingshotPosition = 1.0;
        }
        else {
            slingshotPosition = 0.3;
        }

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        max = Math.max(max, Math.abs(armMotorPower));
        max = Math.max(max, Math.abs(wristMotorPower));
        max = Math.max(max, Math.abs(clawMotorPower));
        max = Math.max(max, Math.abs(slingshotPosition));
        max = Math.max(max, Math.abs(hangMotorPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
            armMotorPower   /= max;
            wristMotorPower /= max;
            clawMotorPower  /= max;
            slingshotPosition /= max;
            hangMotorPower /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        armMotor.setPower(armMotorPower);
        wristMotor.setPosition(wristMotorPower);
        clawMotor1.setPosition(clawMotorPower);
        clawMotor2.setPosition(clawMotorPower);
        slingshotRelease.setPosition(slingshotPosition);
        hangMotor.setPower(hangMotorPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Arm", "%4.2f", armMotorPower);
        telemetry.addData("Wrist", "%4.2f", wristMotorPower);
        telemetry.addData("Claw", "%4.2f", clawMotorPower);
        telemetry.addData("Flywheel", "%4.2f", slingshotPosition);
        telemetry.addData("HangMotor", "%4.2f", hangMotorPower);
        telemetry.update();
    }

    /*
    private void initDoubleVision() {
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
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()
     */

}
