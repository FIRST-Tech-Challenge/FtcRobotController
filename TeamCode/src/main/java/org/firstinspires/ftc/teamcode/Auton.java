package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.nio.ReadOnlyBufferException;
import java.util.List;

@Autonomous(name="test")
public class Auton extends LinearOpMode {
    MecanumChassis robot = new MecanumChassis();

    private ElapsedTime runtime = new ElapsedTime();
    private PositionEstimation positionEstimation = null;
    private PositionControl positionControl = null;
    private volatile double[] robotPos = new double[3];
    private volatile double[] targetPos = new double[3];
    private static final String VUFORIA_KEY =
            "ASOYVAj/////AAABmeLWppvv2E3aulir9L58q2c9jnovtgBUKGOLf6fQRRl2Gmimfb6klxUTuUN4LrMvt1f67Z30a8JuLdxFRlq0VUETIh1E4MUANlTcBWjIT5fg8XYN5C/zIenRIy70ABp5uZ1XlbaWQ9jz38leD/fPbed0WjSN+D6Nmkv9FkcInu8tbv16uB8uWXMUEBcYAnejcYvys1ohlAdc6s1+sWI0QXSawYUOHQoV1hsmY6WpysBbGYv3lQFprY9AyBT69A9ju78WZAm4KAXGugGnD9n1wWMtIJfxo4BYfFtTFJNFI7nnv0EyRB6eZPkq0ScbcYP/z3MoKcFixKvrT9Q47TEI9VZeyLK9GujKZxTIM5PtELbt";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    int[] armPos = {1,2,3};
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            // test magnification
            tfod.setZoom(2, 16.0/9.0);
        }

        this.positionEstimation = new PositionEstimation(robot);
        this.positionControl = new PositionControl(robot, this.positionEstimation);
        this.positionControl.stopRobot();
        telemetry.addData("Status", "Initialized");


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        // Send telemetry message to indicate successful Encoder reset
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

        int DuckPos = 3;
        /*
        for(int k = 0; k < 200;k++) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
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
        }
        */

        goToWayPoint(0.08,0,0,0.7,0,0.01,2);


        goToWayPoint(0.095,1.4,-90,0.7,30,0.01,1);
        setPower(0,0,0,0);
        sleep(1000);
        Duck();
        /*
        goToWayPoint(0.03,1,-90,0.7,0,0.01,1);



        setPower(0.7,0.6,0.7,0.6);
        sleep(1000);
        setPower(0,0,0,0);

        setPower(0.3,0.2,0.3,0.2);
        while(robot.colorSensor.red() + robot.colorSensor.blue() +robot.colorSensor.green() < 2*200){ }
        setPower(0,0,0,0);

        //pickup



*/

    }
    private void goToWayPoint(double x, double y, double angle, double vel, double vw, double disRes, double angleRes) throws InterruptedException {
        targetPos[0] = x;//1.5;  //x
        targetPos[1] = y;//-0.6;   //y
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
    public void setPower(double leftPowerF, double rightPowerF, double leftPowerR, double rightPowerR){

        this.robot.rightFrontDrive.setPower(rightPowerF);
        this.robot.rightRearDrive.setPower(rightPowerR);
        this.robot.leftRearDrive.setPower(leftPowerR);
        this.robot.leftFrontDrive.setPower(leftPowerF);
    }

    private void Extend(int x, int margin ){
        robot.lift.setTargetPosition(x);
        while( Math.abs(robot.lift.getCurrentPosition() - x ) >margin ){

        }
        robot.arm.setPower(0);
    }
    private void Tilt(int x, int margin) {
        robot.arm.setTargetPosition(x);
        while (Math.abs(robot.arm.getCurrentPosition() - x) > margin) {

        }
        robot.arm.setPower(0);
    }

    private int detectPosition(){
        return 0;

    }
    private void Duck(){
        robot.duck.setPower(-0.3);
        sleep(2500);
        robot.duck.setPower(0);
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }


}