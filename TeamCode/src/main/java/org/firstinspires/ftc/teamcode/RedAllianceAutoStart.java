package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

import java.util.List;

/**
 * Created by Sarthak on 10/4/2019.
 */
@Autonomous(name = "Red Alliance Auto Start")
public class RedAllianceAutoStart extends LinearOpMode {

    private MecanumDrivebase mecanumDrivebase = new MecanumDrivebase();

    //Drive motors
    //DcMotor right_front, right_back, left_front, left_back;

    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = (8192/5.93687);
    ElapsedTime Timer;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    double _odometryYPos;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    //String rfName = "FR", rbName = "BR", lfName = "FL", lbName = "BL";
    String verticalLeftEncoderName = "LftOdometry", verticalRightEncoderName = "Intake", horizontalEncoderName = "CntrOdometry";

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    private static final String VUFORIA_KEY =
            " Ac/2h37/////AAABmbXAvaZQqkPSlZv4583jp15xBpCuzySKMfid1ppM+8fZbZsGd93ri87TKmjKKCYA64DjBiSRboJvg0eldCw/QzbXtH/gNzdbd90bD226N+MA3p3b4CH+C8Pe+Q2SPV5d4e23K514g/DZGu5JEHHH5kl1guWLfc485PCIGE/wlhIprwSQmGM535rO6oif8Dka9K6zFPkiiSvsj4SoTdVJ9EMPnSYT1LNRUtcWWyN0aCVFJ2cmU2lCAtvS6t7GACGTQAbq+vURBnS0BLwkqgebDbvPPM6y4LOG904dFosYxQsSJw51CCTDNLXlunkQcEzp8DSjH79jiTb6BMwGtpRbFhyGrtSq+ugYlE6uf+C7V913 ";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setClippingMargins(0,250,0,0);
            //                      L T R B
        }

        mecanumDrivebase.initialize(this);
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        //initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        initDriveHardwareMap(verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        //horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        CameraDevice.getInstance().setFlashTorchMode(true);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        double time = 0.0;
        boolean ringsRecognized = false;
        double ringType = -1; //-1 = unsure, 0 = no rings, 1 = single ring, 4 = quad (4) rings

        Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Timer.reset();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        while (ringsRecognized != true && time < 2525) {
            time = Timer.milliseconds();

            if (time > 2500) {
                ringType = 0; //No ring seen
                _odometryYPos = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            }

            if ((time > 1500) && (time < 2525)) {
                mecanumDrivebase.stop();
                _odometryYPos = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            } else {
                mecanumDrivebase.FR.setPower(0.15);
                mecanumDrivebase.FL.setPower(0.15);
                mecanumDrivebase.BR.setPower(0.15);
                mecanumDrivebase.BL.setPower(0.15);
                _odometryYPos = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            }
            _odometryYPos = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    if (recognition.getLabel() == "Single") {
                        ringType = 1;
                    } else if (recognition.getLabel() == "Quad") {
                        ringType = 4;
                    }
                    ringsRecognized = true;
                }
            }

            telemetry.addData("Time (Milliseconds)", time);
            telemetry.addData("Ring Type", ringType);
            telemetry.addData("Odometry Y Coordinate", _odometryYPos);
            telemetry.update();
        }

        mecanumDrivebase.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumDrivebase.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumDrivebase.FR.setDirection(DcMotorSimple.Direction.REVERSE);

        _odometryYPos = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        sleep (500);

        goToPosition(0*COUNTS_PER_INCH, 24*COUNTS_PER_INCH, 0.25, 0, 1.5*COUNTS_PER_INCH);
        goToPosition(15*COUNTS_PER_INCH,24*COUNTS_PER_INCH,0.25, 0, 1.5*COUNTS_PER_INCH);

        

        mecanumDrivebase.stop();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("Ring Type", ringType);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Odometry Y Pos", _odometryYPos);

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while (opModeIsActive() && distance > allowableDistanceError){
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget,distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            mecanumDrivebase.FR.setPower(-(robot_movement_y_component + (-(pivotCorrection / 180) - robot_movement_x_component)));
            mecanumDrivebase.BR.setPower(robot_movement_y_component + -(pivotCorrection / 180) + robot_movement_x_component);
            mecanumDrivebase.FL.setPower(robot_movement_y_component + (pivotCorrection / 180) + robot_movement_x_component);
            mecanumDrivebase.BL.setPower(robot_movement_y_component + (pivotCorrection / 180) - robot_movement_x_component);

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.update();
        }
    }

    private void initDriveHardwareMap(String vlEncoderName, String vrEncoderName, String hEncoderName){

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

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
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}