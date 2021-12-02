package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.List;

import static java.lang.Math.toRadians;

@Autonomous(name = "BLUE DUCK - STORAGE", group = "Competition")
public class Mecanum_Auto_BlueDuck_Storage extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private DcMotor Spin = null;
    private DcMotor Slide = null;
    private Servo Rotate = null;
    private Servo Push = null;
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;
    //Vuforia setup for vision
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            Robot4100Common.VUFORIA_LICENSE;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Spin = hardwareMap.get(DcMotor.class, "Spin");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide.setDirection(DcMotor.Direction.FORWARD);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Spin.setDirection(DcMotor.Direction.FORWARD);
        Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Rotate = hardwareMap.get(Servo.class, "Rotate");
        Rotate.setDirection(Servo.Direction.FORWARD);
        Push = hardwareMap.get(Servo.class, "Push");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialize position
        Push.setPosition(0.4);
        Slide.setPower(0.15);
        sleep(100);
        Slide.setPower(0.0);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rotate.setPosition(0.03);

        //Vision
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        //Variables
        double center = -1;
        int initialHeight = Slide.getCurrentPosition();
        String visionResult = null;
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //Traj
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-41, 62.125, toRadians(90));
        drive.setPoseEstimate(startPose);
        Trajectory myTrajectory1 = drive.trajectoryBuilder(startPose,true)
                .splineToConstantHeading(new Vector2d(-11.875, 43), toRadians(-90))
                .build();

        waitForStart();
        if(opModeIsActive()) {

            ElapsedTime recogTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            runtime.reset();
            if (visionResult == null) {
                recogTime.reset();
            }

            while (recogTime.milliseconds() <= 2000.0) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            if (recognition.getLabel().equals("Duck")) {
                                center = (recognition.getLeft() + recognition.getRight()) / 2.0;
                            }
                            i++;
                        }
                        telemetry.update();
                    }
                }
            }
            if (center < 0) {
                visionResult = "LEFT";
            } else if (center < 321.85) {
                visionResult = "MIDDLE";
            } else {
                visionResult = "RIGHT";
            }
            telemetry.addLine(visionResult);
            telemetry.update();

            //MOTION TO DUCK
            Trajectory duckTraj = drive.trajectoryBuilder(startPose,true)
                    .splineToLinearHeading(new Pose2d(-64, 54, toRadians(90)), toRadians(0))
                    .build();
            drive.followTrajectory(duckTraj);
            sleep(500);
            Pose2d wall = new Pose2d(-63.88, 53.25, Math.toRadians(90));
            drive.setPoseEstimate(wall);

            //ROTATE DUCK
            drive.setMotorPowers(0.1, 0.1,0.1,0.1);
            sleep(500);
            drive.setMotorPowers(0, 0,0,0);
            Spin.setPower(0.5);
            sleep(3500);
            Spin.setPower(0.0);

            //MOTION TO PLATE
            Trajectory plateTraj1 = drive.trajectoryBuilder(duckTraj.end(),true)
                    .splineToLinearHeading(new Pose2d(-59, 23.75, toRadians(180)), toRadians(-90))
                    .build();
            drive.followTrajectory(plateTraj1);
            Trajectory plateTraj2 = drive.trajectoryBuilder(plateTraj1.end(),true)
                    .back(26.375)
                    .build();
            drive.followTrajectory(plateTraj2);

            //ROTATE
            Rotate.setPosition(1.0);
            sleep(800);

            //SLIDE UP
            if (visionResult == "LEFT") {
                Slide.setTargetPosition(initialHeight);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            } else if (visionResult == "MIDDLE") {
                Slide.setTargetPosition(initialHeight + 700);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            } else if (visionResult == "RIGHT") {
                Slide.setTargetPosition(initialHeight + 1500);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            }
            sleep(600);

            //CLOSER TO PLATE
            Trajectory closeTraj = drive.trajectoryBuilder(plateTraj2.end())
                    .back(1)
                    .build();
            drive.followTrajectory(closeTraj);

            //DUMP AND SLIDE DOWN
            Push.setPosition(0.0);
            sleep(300);
            Push.setPosition(0.4);
            sleep(500);
            Rotate.setPosition(0.03);
            sleep(500);
            Slide.setTargetPosition(initialHeight);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.8);

            //BACK TO STORAGE UNIT & PARK
            Trajectory parkTraj1 = drive.trajectoryBuilder(closeTraj.end())
                    .forward(29)
                    .build();
            drive.followTrajectory(parkTraj1);
            Trajectory parkTraj2 = drive.trajectoryBuilder(parkTraj1.end())
                    .strafeRight(13)
                    .build();
            drive.followTrajectory(parkTraj2);
            sleep(500);

            PoseStorage.currentPose = parkTraj2.end();
            PoseStorage.state = driveMethod.poseState.BLUE;
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
        tfodParameters.minResultConfidence = 0.60f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }



}