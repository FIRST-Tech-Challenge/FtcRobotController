package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;

@Autonomous

public class AllTestAuto extends LinearOpMode {

    // drive motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    // other motors
    private DcMotor bucket;
    private DcMotor bucketTurner;
    private DcMotor linearSlide;
    private DcMotor carouselTurner;
    
    // servo
    private Servo Marker;
    
    // private DigitalChannel limitSwitch;
    
    // distance sensors
    private DistanceSensor FrontRightDistance;
    private DistanceSensor FrontLeftDistance;
    private DistanceSensor BackRightDistance;
    private DistanceSensor BackLeftDistance;
    private DistanceSensor CarouselDistance;
    private DistanceSensor FrontDistance;
    
    //color sensors
    private ColorSensor FrontColor;
    private ColorSensor BackColor;

    
    //object detection variables
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/10820model.tflite";
    private static final String[] LABELS = {
      "10820marker"
    };
    
   /* 
    private static final String TFOD_MODEL_ASSET = "10820marker.tflite";
    private static final String[] LABELS = {
      "neutral",
      "10820marker"
    };
    */
    
    
    private static final String VUFORIA_KEY =
            "Af2A/t3/////AAABmSsJTGsI6Ebgr6cIo4YGqmCBxd+lRenqxeIeJ3TQXcQgRlvrzKhb44K7xnbfJnHjD6eLQaFnpZZEa1Vz1PRYMNj3xCEhYZU7hAYQwyu1KBga3Lo0vEPXPSZW1o8DrM2C6IhYYGifzayZFNwZw5HtnPbyZvJfG4w6TX4EO8F0VSnZt87QtBW27nh5vSgRLN1XdzrVzm8h1ScZrPsIpSKJWVmNCWqOOeibloKfoZbhZ5A8vFz0I3nvMdi/v54DwcmS7GS/hryCgjhy4n9EhD1SnJ5325jnoyi4Fa5a/pibxPmAi8kU7ioHucmQRgv3yQHh17emqait9QNS4jTu6xyM6eeoVADsXTG4f7KK6nlZZjat";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    int objectsRecognized = 0;
    int level = 0;
    int xPosMarker = 150;
    
    @Override
    public void runOpMode() {
        
        
        if (initializeRobot() == 1) {
            telemetry.addData("Status", "Initialization failed..") ;
            return ;
        }
        
        
        
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            
            telemetry.addData("Status", "Running");
            telemetry.update();

            sleep(2000) ;
            
            double pval = 0.5 ;
            
            //testBucket(pval) ;
            //testBucketTurner(pval) ;
            //testCarouselTurner(pval) ;
            //testLimitSwitch();
            
            //linearSlideEncoder(0.5, 100);
            
            
            //strafeRobotLeftEncoders(0.75, 500);
            
            //getSensorValues();
            
            objectDetection();
            
            moveBucketTurnerBackwardEncoders(0.5, 100);
            strafeRobotLeftEncoders(0.75, 1500);
            
            moveRobotForwardEncoders(1, 600);

            releaseItem();
            
            
            sleep(20000) ;
        }
    }
    
    
    private int initializeRobot() {
        
        frontLeft = hardwareMap.get(DcMotor.class,"FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"FrontRight");
        backLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        backRight = hardwareMap.get(DcMotor.class,"BackRight");
        
        frontLeft.setDirection(DcMotor.Direction.FORWARD) ;
        frontRight.setDirection(DcMotor.Direction.REVERSE) ;
        backLeft.setDirection(DcMotor.Direction.FORWARD) ;
        backRight.setDirection(DcMotor.Direction.REVERSE) ;
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
 
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;

        bucket = hardwareMap.get(DcMotor.class,"Bucket");
        bucketTurner = hardwareMap.get(DcMotor.class, "BucketTurner" );
        linearSlide = hardwareMap.get(DcMotor.class, "LinearSlide");
        carouselTurner = hardwareMap.get(DcMotor.class, "CarouselTurner" );

        // limitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch" );
        //limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        
        FrontRightDistance = hardwareMap.get(DistanceSensor.class, "FrontRightDistance");
        FrontLeftDistance = hardwareMap.get(DistanceSensor.class, "FrontLeftDistance");
        BackRightDistance = hardwareMap.get(DistanceSensor.class, "BackRightDistance");
        BackLeftDistance = hardwareMap.get(DistanceSensor.class, "BackLeftDistance");
        FrontDistance = hardwareMap.get(DistanceSensor.class, "FrontDistance");
        CarouselDistance = hardwareMap.get(DistanceSensor.class, "CarouselDistance");
        
        FrontColor = hardwareMap.get(ColorSensor.class, "FrontColor");
        BackColor = hardwareMap.get(ColorSensor.class, "BackColor");
        
        carouselTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        linearSlide.setDirection(DcMotor.Direction.REVERSE) ;
        //positive power moves it up ; negative power down 
        
        bucket.setDirection(DcMotor.Direction.FORWARD) ;
        //positive power picks up ; negative power releases 

        bucketTurner.setDirection(DcMotor.Direction.FORWARD) ;
        //positive power moves it up ; negative power down 
        
        carouselTurner.setDirection(DcMotor.Direction.FORWARD) ;
        //positive power moves it up ; negative power down 
       
       
       
        // object detection initialization
        
        if (initVuforia() == 1) 
            return 1 ;
            
        initTfod();

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
            // (typically 16/9).
            tfod.setZoom(1, 8.0/4.0);
        }
        
        return 0 ;
    }
    
    private void testEachWheel(double pval) {
        
        int start_enc_val, end_enc_val ; 
        
        start_enc_val = frontLeft.getCurrentPosition() ;
        frontLeft.setPower(pval) ;
        sleep(5000) ;
        frontLeft.setPower(0.0) ;
        sleep(1000) ;
        end_enc_val = frontLeft.getCurrentPosition() ;
        telemetry.addData("Front Left", " " + (end_enc_val - start_enc_val) );
        //telemetry.update();
       
        start_enc_val = frontRight.getCurrentPosition() ;
        frontRight.setPower(pval) ;
        sleep(5000) ;
        frontRight.setPower(0.0) ;
        sleep(1000) ;
        end_enc_val = frontRight.getCurrentPosition() ;
        telemetry.addData("Front Right", " " + (end_enc_val - start_enc_val) );
        //telemetry.update();


        start_enc_val = backLeft.getCurrentPosition() ;
        backLeft.setPower(pval) ;
        sleep(5000) ;
        backLeft.setPower(0.0) ;
        sleep(1000) ;
        end_enc_val = backLeft.getCurrentPosition() ;
        telemetry.addData("Back Left", " " + (end_enc_val - start_enc_val) );
        //telemetry.update();
       
       
        start_enc_val = backRight.getCurrentPosition() ;
        backRight.setPower(pval) ;
        sleep(5000) ;
        backRight.setPower(0.0) ;
        sleep(1000) ;
        end_enc_val = backRight.getCurrentPosition() ;
        telemetry.addData("Back Right", " " + (end_enc_val - start_enc_val) );
        telemetry.update();
    }
    
    private void testStrafing(double pval) {
        frontLeft.setPower(pval) ;
        backRight.setPower(pval) ;
        frontRight.setPower(-1.0*pval) ;
        backLeft.setPower(-1.0*pval) ;
        
        sleep(2000) ;
        frontLeft.setPower(0.0) ;
        frontRight.setPower(0.0) ;
        backLeft.setPower(0.0) ;
        backRight.setPower(0.0) ;
        sleep(1000) ;
    }

    private void testBucket(double pval) {
        bucket.setPower(pval) ;
        sleep(5000) ;
        bucket.setPower(0.0) ;
        sleep(1000) ;
        
        bucket.setPower(-1.0*pval) ;
        sleep(5000) ;
        bucket.setPower(0.0) ;
        sleep(1000) ;
    }
    
    private void testBucketTurner(double pval) {
        bucketTurner.setPower(pval) ;
        sleep(1000) ;
        bucketTurner.setPower(0.0) ;
        sleep(1000) ;
        
        bucketTurner.setPower(-1.0*pval) ;
        sleep(1000) ;
        bucketTurner.setPower(0.0) ;
        sleep(1000) ;
    }
    
    
    private void testLinearSlide(double pval) {
        linearSlide.setPower(pval) ;
        sleep(1000) ;
        linearSlide.setPower(0.0) ;
        sleep(1000) ;
        
        linearSlide.setPower(-1.0*pval) ;
        sleep(1000) ;
        linearSlide.setPower(0.0);
        sleep(1000) ;
    }
    
    private void testCarouselTurner(double pval) {
        carouselTurner.setPower(pval) ;
        sleep(5000) ;
        carouselTurner.setPower(0.0) ;
        sleep(1000) ;
        
        carouselTurner.setPower(-1.0*pval) ;
        sleep(5000) ;
        carouselTurner.setPower(0.0) ;
        sleep(1000) ;
    }
    
    /*
    private void testLimitSwitch() {
        
        while (opModeIsActive()) {
         
            if (limitSwitch.getState() == false) {
            
            telemetry.addData("Limit Switch Status: ", "Reached Limit" );
            telemetry.update();
            
            } else if (limitSwitch.getState() == true) {
            
            telemetry.addData("Limit Switch Status: ", "Not Reached Yet" );
            telemetry.update();
            
            } 
            
        }
        
    }
    */
    
    private void linearSlideEncoder(double pval, int enCount) {
        
        linearSlide.setTargetPosition(enCount);
        
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        telemetry.addData("Motor Status: ", "Running" );
        telemetry.update();
        
        linearSlide.setPower(pval);
        
        
        
        
    }
    
    private int initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int vu_err = 0 ; 
        
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        
        try {
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Camera");
        }
        
        catch (Exception e) {
            telemetry.addData("System Error", "Error during hardware get Camera") ;
            vu_err = 1 ;
        }

        if (vu_err == 0) {
            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        } 
        
        return vu_err ;
        

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
       tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
    
    private void objectDetection() {
        
        float leftVal = 0;
        
        if (opModeIsActive()) {
                
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
                                    
                            objectsRecognized ++;
                            leftVal = recognition.getLeft();
                        }
                        
                        if (leftVal <= xPosMarker && objectsRecognized == 1) {
                            
                            level = 3;
                            telemetry.addData("Level", level );
                            telemetry.update();
                            
                        } else if (leftVal >= xPosMarker && objectsRecognized == 1) {
                            
                            level = 2;
                            telemetry.addData("Level", level );
                            telemetry.update();
                            
                        } else if (objectsRecognized == 0) {
                            
                            level = 1;
                            telemetry.addData("Level", level );
                            telemetry.update();
                            
                        }
                      
                      telemetry.update();
                }
            }
        }
        
        
        
    }
    
    
    private void strafeToCarousel() {
        
        
        
        
        
        
    }
    
    private void moveRobotForwardEncoders(double pval, int enCount) {
        
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontRight.setTargetPosition(enCount);
        backRight.setTargetPosition(enCount);
        frontLeft.setTargetPosition(enCount);
        backLeft.setTargetPosition(enCount);
        
        
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        telemetry.addData("Motor Status: ", "Running" );
        telemetry.update();
        
        frontRight.setPower(pval);
        backRight.setPower(pval);
        frontLeft.setPower(pval);
        backLeft.setPower(pval);
        
        while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            
            idle();
            
            
        }
        
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        
    }
    
    private void moveRobotBackwardEncoders(double pval, int enCount) {
        
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontRight.setTargetPosition(enCount * -1);
        backRight.setTargetPosition(enCount * -1);
        frontLeft.setTargetPosition(enCount * -1);
        backLeft.setTargetPosition(enCount * -1);
        
        
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        telemetry.addData("Motor Status: ", "Running" );
        telemetry.update();
        
        frontRight.setPower(pval * -1.0);
        backRight.setPower(pval * -1.0);
        frontLeft.setPower(pval * -1.0);
        backLeft.setPower(pval * -1.0);
        
        while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            
            idle();
            
            
        }
        
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        
    }
    
    private void strafeRobotRightEncoders(double pval, int enCount) {
        
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontRight.setTargetPosition(enCount * -1);
        backRight.setTargetPosition(enCount);
        frontLeft.setTargetPosition(enCount);
        backLeft.setTargetPosition(enCount * -1);
        
        
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        telemetry.addData("Motor Status: ", "Running" );
        telemetry.update();
        
        frontRight.setPower(pval * -1.0);
        backRight.setPower(pval);
        frontLeft.setPower(pval);
        backLeft.setPower(pval * -1.0);
        
        while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            
            idle();
            
            
        }
        
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        
    }
    
    private void strafeRobotLeftEncoders(double pval, int enCount) {
        
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontRight.setTargetPosition(enCount);
        backRight.setTargetPosition(enCount * -1);
        frontLeft.setTargetPosition(enCount * -1);
        backLeft.setTargetPosition(enCount);
        
        
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        telemetry.addData("Motor Status: ", "Running" );
        telemetry.update();
        
        frontRight.setPower(pval);
        backRight.setPower(pval * 1.0);
        frontLeft.setPower(pval * 1.0);
        backLeft.setPower(pval);
        
        while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            
            idle();
            
            
        }
        
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        
    }
    
    private void getSensorValues () {
        
        while (opModeIsActive()) {
            
        telemetry.addData("Front Right Distance: ", FrontRightDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Front Left Distance: ", FrontLeftDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Back Right Distance: ", BackRightDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Back Left Distance: ", BackLeftDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Front Distance: ", FrontDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Carousel Distance: ", CarouselDistance.getDistance(DistanceUnit.INCH));
        
        telemetry.addData("FrontColor Value: ", FrontColor.alpha());
        telemetry.addData("BackColor Value: ", BackColor.alpha() );
        
        telemetry.update();
        
        }
        
        
    }
    
    private void moveBucketTurnerForwardEncoders(double pval, int enCount) {
        
        
        
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        bucketTurner.setTargetPosition(enCount);
        
        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        bucketTurner.setPower(pval);

        while (bucketTurner.isBusy()) {
            
            idle();
            
            
        }
        
        bucketTurner.setPower(0);
        
        
    }
    
    private void moveBucketTurnerBackwardEncoders(double pval, int enCount) {
        
        
        
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        bucketTurner.setTargetPosition(enCount * -1);
        
        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        bucketTurner.setPower(pval * -1);

        while (bucketTurner.isBusy()) {
            
            idle();
            
            
        }
        
        bucketTurner.setPower(0);
        
        
    }
    
    private void moveBucketForwardEncoders(double pval, int enCount) {
        
        
        
        bucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        bucket.setTargetPosition(enCount);
        
        bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        bucket.setPower(pval);

        while (bucket.isBusy()) {
            
            idle();
            
            
        }
        
        bucketTurner.setPower(0);
        
        
    }
    
    private void moveBucketBackwardEncoders(double pval, int enCount) {
        
        
        
        bucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        bucket.setTargetPosition(enCount * -1);
        
        bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        bucket.setPower(pval * -1);

        while (bucket.isBusy()) {
            
            idle();
            
            
        }
        
        bucket.setPower(0);
        
        
    }
    
    private void moveLinearSlideForwardEncoders(double pval, int enCount) {
        
        
        
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        linearSlide.setTargetPosition(enCount);
        
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        linearSlide.setPower(pval);

        while (linearSlide.isBusy()) {
            
            idle();
            
            
        }
        
        linearSlide.setPower(0);
        
        
    }
    
    private void moveLinearSlideBackwardEncoders(double pval, int enCount) {
        
        
        
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        linearSlide.setTargetPosition(enCount * -1);
        
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        linearSlide.setPower(pval * -1);

        while (linearSlide.isBusy()) {
            
            idle();
            
            
        }
        
        linearSlide.setPower(0);
        
        
    }
    
    private void releaseItem() {
        
        linearSlide.setPower(1);
        sleep(500);
        
        
        moveRobotForwardEncoders(1, 200);
        
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        bucketTurner.setTargetPosition(100);
        
        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        bucketTurner.setPower(0.5);
        
        while (bucketTurner.isBusy()) {
            
            idle();
            
            
        } 
        
        bucket.setPower(-1);
        sleep(1000);
        
        
        
        
    }
}
