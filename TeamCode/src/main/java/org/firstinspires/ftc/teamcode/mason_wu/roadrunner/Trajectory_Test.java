package org.firstinspires.ftc.teamcode.mason_wu.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.roadrunner.drive.*;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

@Autonomous(name = "Trajectory Test", group = "4100")
public class Trajectory_Test extends LinearOpMode {
    // Declare OpMode members.
    private DcMotor Arm = null;
    private Servo Hand = null;
    private DcMotor Intake = null;
    private DcMotor Shooter = null;
    private Servo Spanker = null;

    final double HAND_CLOSE_POSITION = 0.9;
    final double HAND_OPEN_POSITION = 0.0;

    //Vuforia setup for vision
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            Robot4100Common.VUFORIA_LICENSE;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize chassis
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize the hardware variables.
        Hand = hardwareMap.get(Servo.class, "hand");
        Arm = hardwareMap.get(DcMotor.class, "arm");

        Intake = hardwareMap.get(DcMotor.class, "intake");
        Spanker = hardwareMap.get(Servo.class, "spanker");
        Shooter = hardwareMap.get(DcMotor.class, "shooter");

        Arm.setDirection(DcMotor.Direction.REVERSE);
        Hand.setPosition(HAND_CLOSE_POSITION);
        //armMotion(true, 0.8, 400);

        Shooter.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Spanker.setPosition(0.85);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        //Initializing vision
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            // tfod.setZoom(2.5, 1.78);
        }

        /////////////// Initial Pose ///////////////
        Pose2d initialPose = new Pose2d();
        //drive.setPoseEstimate(initialPose);
        /////////////// Trajectories ///////////////
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(10)
                .build();

        Trajectory trajAlt = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 10), 0)
                .build();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //Two straight paths
            //drive.followTrajectory(traj1);
            //sleep(200);
            //drive.followTrajectory(traj2);

            //One curvy trajectory
            drive.followTrajectory(trajAlt);
        }
    }
    //Vision
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.4f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }

}