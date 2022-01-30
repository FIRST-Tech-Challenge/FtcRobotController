package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "yxorcursed")
public class Autonofrcursed extends LinearOpMode implements Runnable {
    MecanumChassis robot = new MecanumChassis();

    private ElapsedTime runtime = new ElapsedTime();
    private PositionEstimation positionEstimation = null;
    private PositionControl positionControl = null;
    private volatile double[] robotPos = new double[3];
    private volatile double[] targetPos = new double[3];
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AVc6Q6P/////AAABmR9qZSEpdkuvg3gY65VM2CBj0qyusYkXggAzzLzeUGiCn6T3w/CDUskaaDtCPphwreE" +
                    "L2wGP/498dRPiEcO/T9e5YokJIGna6LI5dIPnbUNe9qZJWbT+ZELQPlHT4/r/ipryTVejft6Agg" +
                    "3zKUvnz9AF1eVFiP/pTmFvijCv2wFyaQD.cBxFmSvaq1YqA2zG0/W79j2wXzr26pJAN6yKZcjspx" +
                    "PYcQm2SLTEuGA/IeNKqV8VTtnLas6OXmEARgF4M0xgM7pgTTs2hzo92q+AacWN6aD95XDmCFFn2" +
                    "fS13tOJRwxWAg/Ju3B5ctPVo+wUFJJ4uWWf5xpc8ArvzzAD+niHQxOvHxa0xi+tbvQOJ12MQ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private final int[] pos = {-375, -775, -1100};

    Autonofrcursed obj = new Autonofrcursed();
    Thread thread = new Thread(obj);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        /*
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }
        */
        this.positionEstimation = new PositionEstimation(robot);
        this.positionControl = new PositionControl(robot, this.positionEstimation);
        this.positionControl.stopRobot();
        telemetry.addData("Status", "Initialized");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        // Send telemetry message to indicate successful Encoder reset

        lift(-200, -0.3);

        telemetry.addData("Path0", "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition());

        telemetry.addData("IMU Mode", "IMU calibrating....");
        telemetry.update();

        //make sure the IMU gyro is calibrated before continue
        while (!isStopRequested() && !robot.imu.isGyroCalibrated() &&
                !robot.imu.isAccelerometerCalibrated() &&
                !robot.imu.isMagnetometerCalibrated() &&
                !robot.imu.isSystemCalibrated()) {
            idle();
        }
        telemetry.addData("IMU Mode", "IMU calibrating done");
        telemetry.update();

        waitForStart();
        int duckPos = 3;
        /*
        // TensorFlow find duck
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                }
                telemetry.update();
            }
        }
        */

        // spin duck +
        this.robot.duck.setPower(-0.25);
        goToWayPoint(-0.5, 0.15, -53, 0.7, 30, 0.01, 1);
        sleep(200);
        // spin duck -


        // drive to team shipping hub +
        lift(pos[duckPos-1],1);
        if(duckPos == 1){

        } else if(duckPos == 2){

        } else {
            thread.start();
            goToWayPoint(0.4, 0.756, -53, 1.5, 30, 0.01, 1);
        }
        // drive to team shipping hub -


        // score preloaded cube +
        // score preloaded cube -


        // drive to warehouse +
        this.robot.duck.setPower(0);
        this.robot.intakeUp.setPower(0);
        goToWayPoint(0.3, 0.55, -53, 2.5, 30, 0.02, 3);
        lift(-50,0.5);
        goToWayPoint(1.6, -0.18, -90,   2.5, 77, 0.01, 1);
        // drive to warehouse -


        // pick up new cube +
        this.robot.intakeUp.setPower(-1.0);
        lift(50,0.3);
        goToWayPoint(2.5, -0.18, -90,   0.7, 30, 0.03, 1);
        sleep(500);
        // pick up new cube


        // drive to team shipping hub +
        goToWayPoint(1.4, -0.13, -90,   2.5, 30, 0.02, 1);
        lift(pos[duckPos-1],1);
        goToWayPoint(1, 0.45, 0, 0.7, 90, 0.01, 1);
        // drive to team shipping hub -


        // score new cube +
        this.robot.intakeUp.setPower(1.0);
        sleep(1500);
        // score new cube -


        // drive to warehouse +
        this.robot.intakeUp.setPower(0);
        goToWayPoint(1.4, -0.18, -90,   2.5, 90, 0.01, 1);
        lift(-50,0.3);
        // drive to warehouse -


        // pick up new cube +
        this.robot.intakeUp.setPower(-1.0);
        lift(50,0.3);
        goToWayPoint(2.7, -0.18, -90,   0.7, 30, 0.03, 1);
        sleep(1000);
        // pick up new cube -


        // drive to team shipping hub +
        goToWayPoint(1.4, -0.18, -90,   1.5, 30, 0.02, 1);
        lift(pos[duckPos-1],1);
        goToWayPoint(1, 0.45, 0, 0.7, 90, 0.01, 1);
        // drive to team shipping hub -


        // score new cube +
        this.robot.intakeUp.setPower(1.0);
        sleep(1500);
        // score new cube -


        // park in warehouse +
        goToWayPoint(1.4, -0.18, -90,   2.5, 90, 0.01, 1);
        lift(-50,0.7);
        goToWayPoint(2.45, -0.18, -90,   1.5, 30, 0.03, 1);
        // park in warehouse -


    }

    public void run(){
        sleep(1000);
        this.robot.intakeUp.setPower(1.0);
        sleep(1000);
        this.robot.intakeUp.setPower(0);
    }

    private void goToWayPoint(double x, double y, double angle, double vel, double vw, double disRes, double angleRes) throws InterruptedException {
        targetPos[0] = (y); // why.
        targetPos[1] = (-x);
        targetPos[2] = angle * Math.PI / 180; // Math.PI /2;   //heading, radian
        this.positionControl.goToTargetPosition(targetPos, vel,vw * Math.PI / 180, disRes,angleRes);
        while(!this.positionControl.checkTaskDone()){
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            robotPos = positionEstimation.getRobotPos();
            telemetry.addData("RobotPos",  "at %5f :%5f:%5f",
                    robotPos[0], robotPos[1], robotPos[2] * 180 / Math.PI);
            telemetry.addData("Debug",  "at %5f",
                    positionControl.debug_task());
            if (this.positionControl.checkTaskDone()){
                telemetry.addData("Task", "Done");
            }
            telemetry.update();
        }
        Thread.sleep(200);

    }

    private void lift(int target, double power){
        this.robot.lift.setTargetPosition(target);
        this.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lift.setPower(power);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
