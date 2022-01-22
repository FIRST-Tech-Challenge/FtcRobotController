package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.Chassis2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.DriveMethod;
import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.FieldConstant;
import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive_Chassis2;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.toRadians;

@Autonomous(name = "RED BARRIER 2", group = "A Competition")
public class Mecanum_Auto_RedBarrier extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private CRServo Spin = null;
    private DcMotor Slide = null;
    private Servo Rotate = null;
    private ArrayList<Double[]> speedList = new ArrayList<Double[]>();
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

    double rotate = 0;
    double speed = 0.5;
    boolean reverse = false;

    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Spin = hardwareMap.get(CRServo.class, "Spin");

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


        Rotate = hardwareMap.get(Servo.class, "Rotate");
        Rotate.setDirection(Servo.Direction.FORWARD);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialize position
        Slide.setPower(0.15);
        sleep(200);
        Slide.setPower(0.0);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rotate.setPosition(0.95);

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
        SampleMecanumDrive_Chassis2 drive = new SampleMecanumDrive_Chassis2(hardwareMap);

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
            Pose2d startPose = FieldConstant.RED_BARRIER_STARTING_POSE;
            drive.setPoseEstimate(startPose);
            if (center < 0) {
                visionResult = "RIGHT";
            } else if (center < 402) {
                visionResult = "LEFT";
                Trajectory plateTraj0 = drive.trajectoryBuilder(startPose,true)
                        .strafeLeft(5)
                        .build();
                drive.followTrajectory(plateTraj0);
                startPose = plateTraj0.end();
            } else {
                visionResult = "MIDDLE";
                Trajectory plateTraj0 = drive.trajectoryBuilder(startPose,true)
                        .strafeLeft(5)
                        .build();
                drive.followTrajectory(plateTraj0);
                startPose = plateTraj0.end();
            }
            telemetry.addLine(visionResult);
            telemetry.update();

            //MOVE MARKER OUT
            Trajectory plateTraj1 = drive.trajectoryBuilder(startPose,true)
                    .back(45)
                    .build();
            drive.followTrajectory(plateTraj1);
            Trajectory plateTraj2 = drive.trajectoryBuilder(plateTraj1.end())
                    .forward(2)
                    .build();
            drive.followTrajectory(plateTraj2);

            //MOTION TO PLATE
            Trajectory plateTraj3 = drive.trajectoryBuilder(plateTraj2.end())
                    .lineToLinearHeading(new Pose2d(7.875, -23.75, toRadians(0)))
                    .build();
            drive.followTrajectory(plateTraj3);

            //ROTATE
            Intake.setPower(0.8);
            sleep(300);
            Intake.setPower(0);
            Rotate.setPosition(0.85);

            //SLIDE UP
            if (visionResult == "LEFT") {
                Slide.setTargetPosition(initialHeight);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            } else if (visionResult == "MIDDLE") {
                Slide.setTargetPosition(initialHeight + 500);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            } else if (visionResult == "RIGHT") {
                Slide.setTargetPosition(initialHeight + 1150);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            }
            sleep(600);

            //CLOSER TO PLATE
            Trajectory closerTraj = drive.trajectoryBuilder(plateTraj3.end())
                    .back(1.5)
                    .build();
            drive.followTrajectory(closerTraj);
            drive.setPoseEstimate(closerTraj.end());

            //DUMP AND SLIDE DOWN
            Rotate.setPosition(0.25);
            sleep(800);
            Rotate.setPosition(0.90);
            sleep(500);
            Slide.setTargetPosition(initialHeight);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.8);
            sleep(500);
            Rotate.setPosition(0.95);

            //BACK TO WALL
            if(visionResult == "LEFT"){
                Trajectory wallTrajR1 = drive.trajectoryBuilder(closerTraj.end())
                        .lineToLinearHeading(new Pose2d(14.8, -23.75, toRadians(-90)))
                        .build();
                drive.followTrajectory(wallTrajR1);
                Trajectory wallTrajR2 = drive.trajectoryBuilder(wallTrajR1.end())
                        .forward(20)
                        .build();
                drive.followTrajectory(wallTrajR2);
                drive.setPoseEstimate(wallTrajR2.end());
            }
            Trajectory wallTraj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(6.5, -65, toRadians(0)))
                    .build();
            drive.followTrajectory(wallTraj1);


            Trajectory wallTraj2 = drive.trajectoryBuilder(wallTraj1.end())
                    .forward(35)
                    .build();
            drive.followTrajectory(wallTraj2);

//            //Collect another block
//            Intake.setPower(0.8);
//            sleep(600);
//            Intake.setPower(0);
//            Rotate.setPosition(1);
//            Intake.setPower(-0.8);
//            sleep(500);
//            Intake.setPower(0);
//            Rotate.setPosition(0.95);
//            Trajectory collTraj1 = drive.trajectoryBuilder(wallTraj2.end())
//                    .back(35)
//                    .build();
//            drive.followTrajectory(collTraj1);
//
//            Trajectory collTraj2 = drive.trajectoryBuilder(collTraj1.end())
//                    .lineToLinearHeading(new Pose2d(-4.25, -40.88, toRadians(-65.99)))
//                    .build();
//            drive.followTrajectory(collTraj2);
//
//            Intake.setPower(0.8);
//            sleep(300);
//            Intake.setPower(0);
//            Slide.setTargetPosition(initialHeight + 1150);
//            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            Slide.setPower(0.8);
//            sleep(600);
//            Rotate.setPosition(0.25);
//            sleep(800);
//            Rotate.setPosition(0.90);
//            sleep(500);
//            Slide.setTargetPosition(initialHeight);
//            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            Slide.setPower(0.8);
//            sleep(500);
//            Rotate.setPosition(0.95);
//
//            Trajectory collTraj3 = drive.trajectoryBuilder(collTraj2.end())
//                    .lineToLinearHeading(new Pose2d(6.5, -65, toRadians(0)))
//                    .build();
//            drive.followTrajectory(collTraj3);
//
//            Trajectory collTraj4 = drive.trajectoryBuilder(collTraj3.end())
//                    .forward(36)
//                    .build();
//            drive.followTrajectory(collTraj4);
//            sleep(300);
//

            PoseStorage.currentPose = drive.getPoseEstimate();
            PoseStorage.state = DriveMethod.poseState.RED;
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