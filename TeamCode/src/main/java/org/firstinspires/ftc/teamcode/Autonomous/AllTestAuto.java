package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    private DcMotor bucket;
    private DcMotor bucketTurner;
    private DcMotor linearSlide;
    private DcMotor carouselTurner;

    private DigitalChannel limitSwitch;
    
    
    //object detection variables
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };
    
    private static final String VUFORIA_KEY =
            "ARAjd5v/////AAABmQ6iQJD2QkgYuX/cCgoLeJtQwAvDgu+2L6atBnCINrvbLCKGuyow1XyTKBZ+OSztPsb0+FJJOwkhD2KL4WI1bjz6+FevU72cCzf9WGwGDvXprFwvbnJV0Il0z2J8y2UNYlukyTIhFKD08b2Rt+0Zv5HWRnvSI6pf5Sbg3WIeQ9v9O4dkki0W0LKz1gYEPpTTOosJO4otCxWvdANs6ZZ21Efr0tFvpR8T0CgbB8EdzRCnnknTglsOsfp03zVSfS7TBLR26QM/kNVF6RBeWMKB1v5juqmohB2tNtdqxL1uQlakmyiY8YmeuPozSyEHYv94cvn6aonUvFw7HLIP4rLe14gsg01I5aHBOizXQtVzShWj";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    
    @Override
    public void runOpMode() {
        
        initializeRobot();
        
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
            
            objectDetection();
            
            sleep(20000) ;
        }
    }
    
    
    private void initializeRobot() {
        
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

        limitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch" );
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        
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
        
        initVuforia();
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
            tfod.setZoom(2.5, 2.0/2.0);
        }
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
    
    private void linearSlideEncoder(double pval, int enCount) {
        
        carouselTurner.setTargetPosition(enCount);
        
        carouselTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        telemetry.addData("Motor Status: ", "Running" );
        telemetry.update();
        
        carouselTurner.setPower(pval);
        
        
        
        
    }
    
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
       tfodParameters.minResultConfidence = 0.8f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    
    private void objectDetection() {
        
        while (opModeIsActive()) {
                
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
        
        
        
    }
    
}
