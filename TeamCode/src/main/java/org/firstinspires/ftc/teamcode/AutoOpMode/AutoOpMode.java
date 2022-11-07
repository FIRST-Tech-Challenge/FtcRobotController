package org.firstinspires.ftc.teamcode.AutoOpMode;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

/**
 * Write a detailed description of the autoClass
 */
//TODO: Copy and Rename Autonomous Mode
@Autonomous(name = "Autonomous World 1", group = "00-Autonomous" , preselectTeleOp = "TeleOp")
public class AutoOpMode extends LinearOpMode{
    public static final Vector2d ORIGIN = new Vector2d(0,0);
    public static final Pose2d ORIGINPOSE = new Pose2d(0,0,Math.toRadians(0));


    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    //Define and declare Robot parking locations
    public enum PARKING_LOCATION{
        LOCATION_1, //Rename to game manual names
        LOCATION_2,
        LOCATION_3
    }
    public static PARKING_LOCATION parkingLocation;

    public enum VISION_IDENTIFIED_TARGET {
        LOCATION_1,
        LOCATION_2,
        LOCATION_3,
    };

    //Select the different colors on the signal sleeve, and define them here
    public enum VISION_IDENTIFIER{
        BOLT,
        BULB,
        PANEL,
        GREEN,
        YELLOW,
        PURPLE
    };
    public static VISION_IDENTIFIER visionIdentifier = VISION_IDENTIFIER.BOLT;


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZME4Mr/////AAABmY+MAyxxT0IileR7JBqaAPsxN2XNYlaGBtEjYaHOlVVTqPQf7NH9eIrosYKKQHPGEXLtJUsdMwZ9e3EXBfy6arulcLPvdpW9bqAB2F2MJJXo35lLA096l/t/LQTi+etVso0Xc5RYkTVSIP3YABp1TeOaF8lCSpjVhPIVW3l/c/XlrnEMPhJk9IgqMEp4P/ifqAqMMMUAIKPEqIrXIv79TvAfdIJig46gfQGaQl5tFHr3nmvMbh/LhFrh5AWAy3B/93cCkOszmYkdHxZStbNB5lMdkTnf3sCnYbQY4jviorfhYrAkqHWH6vNOB9lUt8dOSeHsDtlk33e/6xQgOCNYFN80anYMp82JNDBFX3oyGliV";
//Add Key for each team

    // Class Members
    //private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    //private VuforiaTrackables targets   = null ;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    public  boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    public VuforiaLocalizer.Parameters parameters;

    public List<VuforiaTrackable> allTrackables;

    //Tensor Flow parameters
    /* Note: This sample uses the all-objects Tensor Flow model (PowerPlay.tflite), which contains
     * the following 3 detectable objects
     *  0: Bolt,
     *  1: Bulb,
     *  2: Panel,
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel",
            "4 Green",
            "5 Yellow",
            "6 Purple"
    };

    public String detectedLabel = "None";


    private TFObjectDetector tfod;
    private List<Recognition> recognitions;
    public VISION_IDENTIFIED_TARGET targetLevelDetected = VISION_IDENTIFIED_TARGET.LOCATION_1;

        public boolean DEBUG_FLAG = true;

        public DriveTrain driveTrain;

        boolean parked = false ;
        public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);;

        public VISION_IDENTIFIED_TARGET targetZone = VISION_IDENTIFIED_TARGET.LOCATION_1;//Set a default vision value
        int targetZoneLevel = 0;

        //double af = ALLIANCE_FACTOR;

        @Override
        public void runOpMode() throws InterruptedException {
            /*Create your Subsystem Objects*/
            driveTrain = new DriveTrain(hardwareMap);


            /* Create Controllers */

            //Key Pay inputs to select Game Plan;
            selectGamePlan();

            // Initiate Camera on Init.
            activateVuforiaTensorFlow();

            buildAuto();

           //autonomousController.runAutoControl();

            telemetry.addData("Waiting for start to be pressed.", "Robot is ready!");
            telemetry.update();

            if (isStopRequested()) return;

            while (!isStopRequested()) {

                //Run Vuforia Tensor Flow
                targetZone = runVuforiaTensorFlow();
                targetZoneLevel = targetZone.ordinal();

                if (!parked) {
                    //autonomousController.runAutoControl();
                }

                if (DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

                //Game Play is pressed
                while (opModeIsActive() && !isStopRequested() && !parked) {
                    gameTimer.reset();

                    deactivateVuforiaTensorFlow();

                    runAuto();
                    parked = true;

                    if (DEBUG_FLAG) {
                        printDebugMessages();
                        telemetry.update();
                    }
                }

            }
        }

        //Initialize any other TrajectorySequences as desired
        TrajectorySequence trajInitToPickAndDropConeToPark;

        //Initialize any other Pose2d's as desired
        Pose2d initPose; //4 different poses
        Pose2d midWayPose; //4 different poses
        Pose2d pickConePose; //4 different poses
        Pose2d dropConePose; //4 different poses
        Pose2d parkPose; //4 different poses

        public void buildAuto(){
            switch(startPosition){
                case BLUE_LEFT:
                    initPose = ORIGINPOSE; //Starting pos when on blue alliance
                    midWayPose = new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                    pickConePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    dropConePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    parkPose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    break;
                case BLUE_RIGHT:
                    initPose = ORIGINPOSE; //Starting pos when on blue alliance
                    midWayPose = new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                    pickConePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    dropConePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    parkPose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    break;
                case RED_LEFT:
                    initPose = ORIGINPOSE; //Starting pos when on blue alliance
                    midWayPose = new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                    pickConePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    dropConePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    parkPose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    break;
                case RED_RIGHT:
                    initPose = ORIGINPOSE; //Starting pos when on blue alliance
                    midWayPose = new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                    pickConePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    dropConePose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    parkPose =  new Pose2d(0,0,Math.toRadians(0)); //Choose the pose to move to the stack of cones
                    break;
            }

            //Pick 5 cones and park
            trajInitToPickAndDropConeToPark = driveTrain.trajectorySequenceBuilder(initPose)
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(pickConePose)
                    .addTemporalMarker(0,()->{
                        pickCone();
                    })
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(dropConePose)
                    .addTemporalMarker(0,()->{
                        dropCone();
                    })
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(pickConePose)
                    .addTemporalMarker(0,()->{
                        pickCone();
                    })
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(dropConePose)
                    .addTemporalMarker(0,()->{
                        dropCone();
                    })
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(pickConePose)
                    .addTemporalMarker(0,()->{
                        pickCone();
                    })
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(dropConePose)
                    .addTemporalMarker(0,()->{
                        dropCone();
                    })
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(pickConePose)
                    .addTemporalMarker(0,()->{
                        pickCone();
                    })
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(dropConePose)
                    .addTemporalMarker(0,()->{
                        dropCone();
                    })
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(pickConePose)
                    .addTemporalMarker(0,()->{
                        pickCone();
                    })
                    .lineToLinearHeading(midWayPose)
                    .lineToLinearHeading(dropConePose)
                    .addTemporalMarker(0,()->{
                        dropCone();
                    })
                    .build();
        }

        public void runAuto(){
            //Write any other actions to take during auto, or any other conditions for maneuvering
            driveTrain.followTrajectorySequence(trajInitToPickAndDropConeToPark);
            return;
        }

        //Write a method which is able to pick the cone depending on your subsystems
        public void pickCone(){

        }

        //Write a method which is able to drop the cone depending on your subsystems
        public void dropCone(){

        }

        public void selectGamePlan() {
            telemetry.setAutoClear(true);
            telemetry.addData("Compile time : ", "22:00 :: 1/27/2022");

            //******select start pose******
            while(!isStopRequested()){
                telemetry.addData("Enter a Start Pose:","");
                telemetry.addData("Blue Left: (X)", "");
                telemetry.addData("Blue Right: (Y)", "");
                telemetry.addData("Red Left: (B)", "");
                telemetry.addData("Red Right: (A)", "");
                if(gamepad1.x){
                    startPosition = START_POSITION.BLUE_LEFT;
                    telemetry.addData("Start Position: ", startPosition);
                    break;
                }
                if(gamepad1.y){
                    startPosition = START_POSITION.BLUE_RIGHT;
                    telemetry.addData("Start Position: ", startPosition);
                    break;
                }
                if(gamepad1.b){
                    startPosition = START_POSITION.RED_LEFT;
                    telemetry.addData("Start Position: ", startPosition);
                    break;
                }
                if(gamepad1.a){
                    startPosition = START_POSITION.RED_RIGHT;
                    telemetry.addData("Start Position: ", startPosition);
                    break;
                }
                telemetry.update();
            }
            telemetry.clearAll();
            telemetry.addData("StartPose : ", startPosition);
            telemetry.update();
        }

        /**
         * Method to add debug messages. Update as telemetry.addData.
         * Use public attributes or methods if needs to be called here.
         */
        public void printDebugMessages(){
            telemetry.setAutoClear(true);
            telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);


            telemetry.addData("Start Position : ", startPosition);
            telemetry.addData("Parking Location : ", parkingLocation);

            //****** Drive debug ******
            telemetry.addData("Drive Mode : ", driveTrain.driveMode);
            telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
            //telemetry.addData("Battery Power", driveTrain.getBatteryVoltage(hardwareMap));

            telemetry.addData("Vision detectedLabel", detectedLabel);
            telemetry.addData("Vision targetZone :", targetZone);
            telemetry.addData("Vision targetZoneLevel :", targetZoneLevel);

            telemetry.addData("Game Timer : ", gameTimer.time());

            telemetry.update();

        }
    /**
     * Initialize the Vuforia localization engine.
     */
    public AutoOpMode(HardwareMap hardwareMap) {
        //if (activeWebcam == ACTIVE_WEBCAM.WEBCAM1){
        webcamName = hardwareMap.get(WebcamName.class, "Webcam1");
        //} /*else { //TODO: Uncomment if using 2 cameras;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam2");
        //}


        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        initTfod(hardwareMap);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320; //320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

    }
    /**
     * Activate Vuforia Tensor Flow to determine target zone
     * This is to be done at Init in Autonomous mode
     */
    public void activateVuforiaTensorFlow(){
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(1.75, 16.0/9.0);
            tfod.setZoom(1.0, 16.0/9.0);
            recognitions = tfod.getUpdatedRecognitions();
        }
    }

    /**
     * Run Tensor flow algorithm.
     * This is to be run till the play button is pressed.. the last target zone identified is returned.
     * @return
     */



    public VISION_IDENTIFIED_TARGET runVuforiaTensorFlow() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                for (Recognition recognition : recognitions) {
                    // check label to see which target zone to go after.
                    detectedLabel = recognition.getLabel();
                    switch(detectedLabel){
                        case "1 Bolt":
                        case "6 Purple":
                            targetLevelDetected = VISION_IDENTIFIED_TARGET.LOCATION_1;
                            break;
                        case "2 Bulb":
                        case "5 Yellow":
                            targetLevelDetected = VISION_IDENTIFIED_TARGET.LOCATION_2;
                            break;
                        case "3 Panel":
                        case "4 Green":
                            targetLevelDetected = VISION_IDENTIFIED_TARGET.LOCATION_3;
                            break;
                    }
                }
            }
        }

        return targetLevelDetected;
    }

    /**
     * Stop Tensor Flow algorithm
     */
    public void deactivateVuforiaTensorFlow(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}

