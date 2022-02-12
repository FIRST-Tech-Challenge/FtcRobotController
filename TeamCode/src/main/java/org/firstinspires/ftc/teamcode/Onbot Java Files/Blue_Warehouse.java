package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous (name = "Blue: Warehouse Side Autonomous", group = "Blue")
public class Blue_Warehouse extends LinearOpMode {
  
  private DcMotorEx frontLeft;
  private DcMotorEx frontRight;
  private DcMotorEx backLeft;
  private DcMotorEx backRight;
  
  private DcMotor bucket;
  private DcMotor bucketTurner;
  private DcMotor armMotor;
  private DcMotor carouselTurner;
  
  private Servo cappingServo;
  
  
  private DistanceSensor rightDistance;
  private DistanceSensor leftDistance;
  private DistanceSensor carouselDistance;
  private DistanceSensor freightDistance;
  
  private ColorSensor frontColor;
  private ColorSensor backColor;
  
  
  private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/10820model.tflite";
  
  private static final String[] LABELS = new String[] { "10820marker" };
  
  private static final String VUFORIA_KEY = "Af2A/t3/////AAABmSsJTGsI6Ebgr6cIo4YGqmCBxd+lRenqxeIeJ3TQXcQgRlvrzKhb44K7xnbfJnHjD6eLQaFnpZZEa1Vz1PRYMNj3xCEhYZU7hAYQwyu1KBga3Lo0vEPXPSZW1o8DrM2C6IhYYGifzayZFNwZw5HtnPbyZvJfG4w6TX4EO8F0VSnZt87QtBW27nh5vSgRLN1XdzrVzm8h1ScZrPsIpSKJWVmNCWqOOeibloKfoZbhZ5A8vFz0I3nvMdi/v54DwcmS7GS/hryCgjhy4n9EhD1SnJ5325jnoyi4Fa5a/pibxPmAi8kU7ioHucmQRgv3yQHh17emqait9QNS4jTu6xyM6eeoVADsXTG4f7KK6nlZZjat";
  
  private VuforiaLocalizer vuforia;
  
  private TFObjectDetector tfod;
  
  int objectsRecognized = 0;
  
  int level = 0;
  
  int xPosMarker = 150;
  
  BNO055IMU             imu;
  Orientation           lastAngles = new Orientation();
  double                globalAngle, power = .30, correction;
  
  public void runOpMode() {
    
    if (initializeRobot() == 1) {
      telemetry.addData("Status", "Initialization failed..");
      return;
    }
    

    telemetry.addData("Status", "Initialized");
    telemetry.update();
    
    waitForStart();
    
    if (opModeIsActive()) {
      
      telemetry.addData("Status", "Running");
      telemetry.update();
      
      double pval = 0.5D;
      
      objectDetection();
      
      telemetry.addData("level" , level );
      telemetry.update();
        

      strafeToShippingHub();
      
      placeFreightOnShippingHub();
      
      //parkWarehouse();
       
      //releaseItemLevel3();
      
      telemetry.addData("status" , "exited" );
      telemetry.update();
      
      frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      
      frontRight.setTargetPosition(-1000);
      backRight.setTargetPosition(-1000);
      frontLeft.setTargetPosition(1000);
      backLeft.setTargetPosition(1000);
      
      frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      
      telemetry.addData("Motor Status: ", "Running");
      telemetry.update();
      
      setThrottle(0.75, -0.75, 0.75, -0.75);
      
      while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
        idle(); 
      
      frontRight.setPower(0.0D);
      backRight.setPower(0.0D);
      frontLeft.setPower(0.0D);
      backLeft.setPower(0.0D);
      
      moveRobotForwardEncoders(0.75, 3000);
        
      
      //rotate(70, 0.75);

      //strafeToCarousel(0.75D);
      
      /*
      while (opModeIsActive()) {
      telemetry.addData("bucket position: ", bucketTurner.getCurrentPosition());
      telemetry.addData("arm position: ", armMotor.getCurrentPosition());
      telemetry.update();
      }
      */
      
    }
    
  }
  
  private int initializeRobot() {
    
    frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
    frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
    backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
    backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
    
    frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    bucket = (DcMotor)hardwareMap.get(DcMotor.class, "Bucket");
    bucketTurner = (DcMotor)hardwareMap.get(DcMotor.class, "BucketTurner");
    armMotor = (DcMotor)hardwareMap.get(DcMotor.class, "ArmMotor");
    carouselTurner = (DcMotor)hardwareMap.get(DcMotor.class, "CarouselTurner");
    
    armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    bucketTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    rightDistance = (DistanceSensor)hardwareMap.get(DistanceSensor.class, "RightDistance");
    leftDistance = (DistanceSensor)hardwareMap.get(DistanceSensor.class, "LeftDistance");
    freightDistance = (DistanceSensor)hardwareMap.get(DistanceSensor.class, "FreightDistance");
    carouselDistance = (DistanceSensor)hardwareMap.get(DistanceSensor.class, "CarouselDistance");
    
    frontColor = (ColorSensor)hardwareMap.get(ColorSensor.class, "FrontColor");
    backColor = (ColorSensor)hardwareMap.get(ColorSensor.class, "BackColor");
    
    carouselTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    carouselTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    bucket.setDirection(DcMotorSimple.Direction.FORWARD);
    bucketTurner.setDirection(DcMotorSimple.Direction.FORWARD);
    carouselTurner.setDirection(DcMotorSimple.Direction.FORWARD);
    
    frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    //initializeIMU();
    
    if (initVuforia() == 1)
      return 1; 
    initTfod();
    if (tfod != null) {
      tfod.activate();
      tfod.setZoom(1.0D, 2.0D);
    } 
    return 0;
  }
  
  private void testEachWheel(double pval) {
    int start_enc_val = backRight.getCurrentPosition();
    backRight.setPower(pval);
    sleep(5000L);
    backRight.setPower(0.0D);
    sleep(1000L);
    int end_enc_val = backRight.getCurrentPosition();
    telemetry.addData("Back Right", " " + (end_enc_val - start_enc_val));
    telemetry.update();
  }
  
  private void testStrafing(double pval) {
    frontLeft.setPower(pval);
    backRight.setPower(pval);
    frontRight.setPower(-1.0D * pval);
    backLeft.setPower(-1.0D * pval);
    sleep(2000L);
    frontLeft.setPower(0.0D);
    frontRight.setPower(0.0D);
    backLeft.setPower(0.0D);
    backRight.setPower(0.0D);
    sleep(1000L);
  }
  
  private void testBucket(double pval) {
    bucket.setPower(pval);
    sleep(5000L);
    bucket.setPower(0.0D);
    sleep(1000L);
    bucket.setPower(-1.0D * pval);
    sleep(5000L);
    bucket.setPower(0.0D);
    sleep(1000L);
  }
  
  private void testBucketTurner(double pval) {
    bucketTurner.setPower(pval);
    sleep(1000L);
    bucketTurner.setPower(0.0D);
    sleep(1000L);
    bucketTurner.setPower(-1.0D * pval);
    sleep(1000L);
    bucketTurner.setPower(0.0D);
    sleep(1000L);
  }
  
  private void testArm(double pval) {
    armMotor.setPower(pval);
    sleep(1000L);
    armMotor.setPower(0.0D);
    sleep(1000L);
    armMotor.setPower(-1.0D * pval);
    sleep(1000L);
    armMotor.setPower(0.0D);
    sleep(1000L);
  }
  
  private void testCarouselTurner(double pval) {
    carouselTurner.setPower(pval);
    sleep(5000L);
    carouselTurner.setPower(0.0D);
    sleep(1000L);
    carouselTurner.setPower(-1.0D * pval);
    sleep(5000L);
    carouselTurner.setPower(0.0D);
    sleep(1000L);
  }
  
  private void armMotorEncoder(double pval, int enCount) {
    armMotor.setTargetPosition(enCount);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    telemetry.addData("Motor Status: ", "Running");
    telemetry.update();
    armMotor.setPower(pval);
  }
  
  private int initVuforia() {
    int vu_err = 0;
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    parameters.vuforiaLicenseKey = "Af2A/t3/////AAABmSsJTGsI6Ebgr6cIo4YGqmCBxd+lRenqxeIeJ3TQXcQgRlvrzKhb44K7xnbfJnHjD6eLQaFnpZZEa1Vz1PRYMNj3xCEhYZU7hAYQwyu1KBga3Lo0vEPXPSZW1o8DrM2C6IhYYGifzayZFNwZw5HtnPbyZvJfG4w6TX4EO8F0VSnZt87QtBW27nh5vSgRLN1XdzrVzm8h1ScZrPsIpSKJWVmNCWqOOeibloKfoZbhZ5A8vFz0I3nvMdi/v54DwcmS7GS/hryCgjhy4n9EhD1SnJ5325jnoyi4Fa5a/pibxPmAi8kU7ioHucmQRgv3yQHh17emqait9QNS4jTu6xyM6eeoVADsXTG4f7KK6nlZZjat";
    try {
      parameters.cameraName = (CameraName)hardwareMap.get(WebcamName.class, "Camera");
    } catch (Exception e) {
      telemetry.addData("System Error", "Error during hardware get Camera");
      vu_err = 1;
    } 
    if (vu_err == 0)
      vuforia = ClassFactory.getInstance().createVuforia(parameters); 
    return vu_err;
  }
  
  private void initTfod() {
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext
        .getPackageName());
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    tfodParameters.minResultConfidence = 0.8F;
    tfodParameters.isModelTensorFlow2 = true;
    tfodParameters.inputSize = 320;
    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromFile("/sdcard/FIRST/tflitemodels/10820model.tflite", LABELS);
  }
  
  private void objectDetection() {
    float leftVal = 0.0F;
    if (opModeIsActive())
      if (tfod != null) {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
          telemetry.addData("# Object Detected", Integer.valueOf(updatedRecognitions.size()));
          int i = 0;
          for (Recognition recognition : updatedRecognitions) {
            telemetry.addData(String.format("label (%d)", new java.lang.Object[] { Integer.valueOf(i) }), recognition.getLabel());
            telemetry.addData(String.format("  left,top (%d)", new java.lang.Object[] { Integer.valueOf(i) }), "%.03f , %.03f", new java.lang.Object[] { Float.valueOf(recognition.getLeft()), Float.valueOf(recognition.getTop()) });
            telemetry.addData(String.format("  right,bottom (%d)", new java.lang.Object[] { Integer.valueOf(i) }), "%.03f , %.03f", new java.lang.Object[] { Float.valueOf(recognition.getRight()), Float.valueOf(recognition.getBottom()) });
            i++;
            objectsRecognized++;
            leftVal = recognition.getLeft();
          } 
          if (leftVal <= xPosMarker && objectsRecognized == 1) {
            level = 2;
            telemetry.addData("Level", Integer.valueOf(level));
            telemetry.update();
          } else if (leftVal >= xPosMarker && objectsRecognized == 1) {
            level = 3;
            telemetry.addData("Level", Integer.valueOf(level));
            telemetry.update();
          } else if (objectsRecognized == 0) {
            level = 1;
            telemetry.addData("Level", Integer.valueOf(level));
            telemetry.update();
          } 
          telemetry.update();
        } 
      }  
  }
  
  private void strafeToCarousel(double pval) {
    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setPower(pval * -1.0D);
    backRight.setPower(pval);
    frontLeft.setPower(pval);
    backLeft.setPower(pval * -1.0D);
    while (carouselDistance.getDistance(DistanceUnit.INCH) >= 15.0D)
      idle(); 
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    moveRobotForwardEncoders(0.75D, 200);
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setTargetPosition(750);
    backRight.setTargetPosition(750);
    frontLeft.setTargetPosition(-750);
    backLeft.setTargetPosition(-750);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    telemetry.addData("Motor Status: ", "Running");
    telemetry.update();
    frontRight.setPower(1.0D);
    backRight.setPower(1.0D);
    frontLeft.setPower(-1.0D);
    backLeft.setPower(-1.0D);
    while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
      idle(); 
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    strafeRobotRightEncoders(0.5D, 300);
    carouselTurner.setPower(1.0D);
    sleep(3500L);
    carouselTurner.setPower(0.0D);
    strafeRobotLeftEncoders(0.5D, 200);
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setTargetPosition(-750);
    backRight.setTargetPosition(-750);
    frontLeft.setTargetPosition(750);
    backLeft.setTargetPosition(750);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    telemetry.addData("Motor Status: ", "Running");
    telemetry.update();
    frontRight.setPower(-1.0D);
    backRight.setPower(-1.0D);
    frontLeft.setPower(1.0D);
    backLeft.setPower(1.0D);
    while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
      idle(); 
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setPower(-0.5D);
    backRight.setPower(0.5D);
    frontLeft.setPower(0.5D);
    backLeft.setPower(-0.5D);
    while (leftDistance.getDistance(DistanceUnit.INCH) >= 5.0D)
      idle(); 
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setPower(0.25D);
    backRight.setPower(0.25D);
    frontLeft.setPower(0.25D);
    backLeft.setPower(0.25D);
    while (backColor.alpha() <= 400)
      idle(); 
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
    bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bucketTurner.setTargetPosition(55);
    bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bucketTurner.setPower(0.75D);
    while (bucketTurner.isBusy())
      idle(); 
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
    telemetry.addData("Motor Status: ", "Running");
    telemetry.update();
    frontRight.setPower(pval);
    backRight.setPower(pval);
    frontLeft.setPower(pval);
    backLeft.setPower(pval);
    while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
      idle(); 
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
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
    telemetry.addData("Motor Status: ", "Running");
    telemetry.update();
    frontRight.setPower(pval * -1.0D);
    backRight.setPower(pval * -1.0D);
    frontLeft.setPower(pval * -1.0D);
    backLeft.setPower(pval * -1.0D);
    while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
      idle(); 
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
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
    telemetry.addData("Motor Status: ", "Running");
    telemetry.update();
    frontRight.setPower(pval * -1.0D);
    backRight.setPower(pval);
    frontLeft.setPower(pval);
    backLeft.setPower(pval * -1.0D);
    while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
      idle(); 
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
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
    telemetry.addData("Motor Status: ", "Running");
    telemetry.update();
    frontRight.setPower(pval);
    backRight.setPower(pval * -1.0D);
    frontLeft.setPower(pval * -1.0D);
    backLeft.setPower(pval);
    while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
      idle(); 
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
  }
  
  private void getSensorValues() {
    while (opModeIsActive()) {
      
      telemetry.addData("freight distance: ", freightDistance.getDistance(DistanceUnit.INCH));
      telemetry.addData("right distance: ", rightDistance.getDistance(DistanceUnit.INCH));
      telemetry.addData("left distance: ", leftDistance.getDistance(DistanceUnit.INCH));
      telemetry.addData("carousel distance: ", carouselDistance.getDistance(DistanceUnit.INCH));
              
      telemetry.addData("front color: ", frontColor.alpha());
      telemetry.addData("back color: ", backColor.alpha());
            
      telemetry.update();
    } 
  }
  
  private void moveBucketTurnerForwardEncoders(double pval, int enCount) {
    bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bucketTurner.setTargetPosition(enCount);
    bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bucketTurner.setPower(pval);
    while (bucketTurner.isBusy())
      idle(); 
    bucketTurner.setPower(0.0D);
  }
  
  private void moveBucketTurnerBackwardEncoders(double pval, int enCount) {
    bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bucketTurner.setTargetPosition(enCount * -1);
    bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bucketTurner.setPower(pval * -1.0D);
    while (bucketTurner.isBusy())
      idle(); 
    bucketTurner.setPower(0.0D);
  }
  
  private void moveBucketForwardEncoders(double pval, int enCount) {
    bucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bucket.setTargetPosition(enCount);
    bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bucket.setPower(pval);
    while (bucket.isBusy())
      idle(); 
    bucketTurner.setPower(0.0D);
  }
  
  private void moveBucketBackwardEncoders(double pval, int enCount) {
    bucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bucket.setTargetPosition(enCount * -1);
    bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bucket.setPower(pval * -1.0D);
    while (bucket.isBusy())
      idle(); 
    bucket.setPower(0.0D);
  }
  
  private void movearmMotorForwardEncoders(double pval, int enCount) {
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setTargetPosition(enCount);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armMotor.setPower(pval);
    while (armMotor.isBusy())
      idle(); 
    armMotor.setPower(0.0D);
  }
  
  private void movearmMotorBackwardEncoders(double pval, int enCount) {
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setTargetPosition(enCount * -1);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armMotor.setPower(pval * -1.0D);
    while (armMotor.isBusy())
      idle(); 
    armMotor.setPower(0.0D);
  }
  
  private void releaseItemLevel1() {
    
    moveRobotForwardEncoders(0.75D, 400);

    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setTargetPosition(-225);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armMotor.setPower(-0.75D);
    while (armMotor.isBusy())
      idle();
    bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bucketTurner.setTargetPosition(750);
    bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bucketTurner.setPower(0.75D);
    while (bucketTurner.isBusy())
      idle(); 
    bucket.setPower(-0.75D);
    sleep(1500L);
    bucket.setPower(0.0D);

    telemetry.addData("Status: ", "Freight Placed");
    telemetry.update();
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
  }
  
  private void releaseItemLevel2() {
    
    moveRobotForwardEncoders(0.75D, 400);

    
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setTargetPosition(-550);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armMotor.setPower(-0.75D);
    while (armMotor.isBusy())
      idle(); 
    bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bucketTurner.setTargetPosition(900);
    bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bucketTurner.setPower(0.75D);
    while (bucketTurner.isBusy())
      idle(); 
    bucket.setPower(-0.75D);
    sleep(1500L);
    bucket.setPower(0.0D);

    /*
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setTargetPosition(-180);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armMotor.setPower(-0.75D);
    while (armMotor.isBusy())
      idle(); 
    armMotor.setPower(0.0D);
    */
    
    telemetry.addData("Status: ", "Freight Placed");
    telemetry.update();
    
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
  }
  
  private void releaseItemLevel3() {
    
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    moveRobotForwardEncoders(0.75D, 600);
    
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
    
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setTargetPosition(-1100);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armMotor.setPower(-0.75D);
    while (armMotor.isBusy())
      idle(); 
    bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bucketTurner.setTargetPosition(1220);
    bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bucketTurner.setPower(0.75D);
    while (bucketTurner.isBusy())
      idle(); 
    bucket.setPower(-0.75D);
    sleep(1500L);
    bucket.setPower(0.0D);
    armMotor.setPower(0);
    bucketTurner.setPower(0);
    
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
    

    telemetry.addData("Status: ", "Freight Placed");
    telemetry.update();
    
    stopAllWheelsPower();
    
    telemetry.addData("Status: ", "reached");
    telemetry.update();
    

    //parkWarehouse();
    
    
  }
  
  private void strafeToShippingHub() {
    
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    frontRight.setTargetPosition(-1500);
    backRight.setTargetPosition(1600);
    frontLeft.setTargetPosition(1500);
    backLeft.setTargetPosition(-1500);
    
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    
    frontRight.setPower(-0.5);
    backRight.setPower(0.5);
    frontLeft.setPower(0.5);
    backLeft.setPower(-0.5);
    
    while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
      idle(); 
    
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
    
  }
  
  private void strafeLeft(double pval, int duration) {
    frontRight.setPower(pval);
    backRight.setPower(pval * -1.0D);
    frontLeft.setPower(pval * -1.0D);
    backLeft.setPower(pval);
    sleep(duration);
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
  }
  
  private void strafeRight(double pval, int duration) {
    frontRight.setPower(pval * -1.0D);
    backRight.setPower(pval);
    frontLeft.setPower(pval);
    backLeft.setPower(pval * -1.0D);
    sleep(duration);
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
  }
  
  private void placeFreightOnShippingHub() {
    if (level == 3) {
      
      telemetry.addData("Placing Freight on Level 3: ", "In Progress");
      telemetry.update();
      releaseItemLevel3();
      
    } else if (level == 2) {
      
      telemetry.addData("Placing Freight on Level 2: ", "In Progress");
      telemetry.update();
      releaseItemLevel2();
      
    } else if (level == 1) {
      
      telemetry.addData("Placing Freight on Level 1: ", "In Progress");
      telemetry.update();
      releaseItemLevel1();
      
    } else {
      
      telemetry.addData("No Objects Detected: ", "Failure");
      telemetry.update();
    } 
    

  }
  
  private void parkWarehouse() {
    
    rotate(75, 0.5);
    
    sleep(100);
    
    moveRobotForwardEncoders(0.75, 3000);
    
  }
  
  private void initializeIMU() {
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    parameters.mode                = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled      = false;    

    // get and initialize the IMU 
    // The imu is assumed to be on I2C port 
    // and configured to be a sensor of type "AdaFruit IMU" and named "imu"
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    imu.initialize(parameters);

    telemetry.addData("Mode", "IMU calibrating...");
    telemetry.update();

    // make sure the imu gyro is calibrated before continuing.
    while (!isStopRequested() && !imu.isGyroCalibrated())
    {
      sleep(50);
      idle();
    }

    telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
    telemetry.update( );
  }

  private void displayOrientationValues(Orientation orient, String name) {
    // ensure that the order is set up correctly as specified in getAngularDistance
    telemetry.addData(name, "Z: " + orient.firstAngle + ", Y: " + orient.secondAngle + ", X: " + orient.thirdAngle);
  }
  
  private void resetAngle() {
    lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    globalAngle = 0;
  }

  private double getAngle() {
        
    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

    if (deltaAngle < -180)
        deltaAngle += 360;
    else if (deltaAngle > 180)
        deltaAngle -= 360;

    globalAngle += deltaAngle;

    lastAngles = angles;

    return globalAngle;
  }

    
  private double checkDirection() {
        
    double correction, angle, gain = .1;

    angle = getAngle();

    if (angle == 0)
        correction = 0;             // no adjustment.
    else
        correction = -angle;        // reverse sign of angle for correction.

    correction = correction * gain;

    return correction;
  }
    
  private void rotate(int degrees, double power) {
    double  leftPower, rightPower;

    // restart imu movement tracking.
    resetAngle();

    // getAngle() returns + when rotating counter clockwise (left) and - when rotating
    // clockwise (right).

    if (degrees < 0)
    {   // turn left
        leftPower = -power;
        rightPower = power;
    }
    else if (degrees > 0)
    {   // turn right
        leftPower = power;
        rightPower = -power;
    }
    else return;

    // set power to rotate.
    frontLeft.setPower(leftPower);
    backLeft.setPower(leftPower);
    frontRight.setPower(rightPower);
    backRight.setPower(rightPower);
    
    // rotate until turn is completed.
    if (degrees < 0)
    {
        // On right turn we have to get off zero first.
        while (getAngle() == 0) {}

        while (getAngle() > degrees) {}
    }
    else    // right turn.
        while (getAngle() < degrees) {}

    // turn the motors off.
    stopAllWheelsPower();
    

    // reset angle tracking on new heading.
    resetAngle();
  }
    
    void stopAllWheelsPower() {
        frontLeft.setPower(0.0);
        backLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backRight.setPower(0.0);
    }    
    
    private void testIMU() {
    // it is assumed here the the IMU has been initialized
    
    // first reset and then get the angles 
    // reset here does not mean IMU reset. just storing it inside lastAngles and 
    // making global angles as zero 

    resetAngle();
    
    /*
    long cur_time = System.currentTimeMillis() ;
    while ((System.currentTimeMillis() - cur_time) < 20000) {
        // display the values 
        getAngle() ;
        displayOrientationValues(lastAngles, "imu vals") ;
        telemetry.update() ;
    }
    */
    getAngle() ;
    displayOrientationValues(lastAngles, "imu vals") ;
    telemetry.update() ;

    rotate(-45,0.75) ;
    //sleep(1000) ;
    
    getAngle() ;
    displayOrientationValues(lastAngles, "imu vals") ;
    telemetry.update() ;
    

    /*    
    // turn the robot  and see what changes take place in the values 
    driveFL.setPower(1.0);
    driveBL.setPower(1.0);
    driveFR.setPower(1.0);
    driveBR.setPower(1.0);
    sleep (1500) ;
    stopAllWheelsPower();
    
    // let it rest for a bit
    sleep(1000) ;

    // display the values again
    getAngle() ;
    displayOrientationValues(lastAngles, "imu end") ;

    telemetry.update() ;
    */
    
    // Experimentally, we have established that the firstAngle value is the 
    // one that changes when the robot changes direction 
    // This is crucial.. 
    // Check REV robotics documentation.. it seems a bit off 
    
    sleep(20000) ;
  }
  
  private void strafeCarousel() {
    
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setTargetPosition(-3000);
    backRight.setTargetPosition(3100);
    frontLeft.setTargetPosition(3000);
    backLeft.setTargetPosition(-3000);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRight.setPower(-0.5D);
    backRight.setPower(0.5D);
    frontLeft.setPower(0.5D);
    backLeft.setPower(-0.5D);
    
    long curTime = System.currentTimeMillis();
    int timeRunning = 4000;
    long desiredTime = curTime + timeRunning;
    
    while (System.currentTimeMillis() <= desiredTime)
      idle(); 
    
    frontRight.setPower(0.0D);
    backRight.setPower(0.0D);
    frontLeft.setPower(0.0D);
    backLeft.setPower(0.0D);
    
    
  }
  
  public void setThrottle(double frontLeftVelocity, double frontRightVelocity, double backLeftVelocity, double backRightVelocity) {
        // Constrain throttle values to between -1.0 to 1.0
        if (frontLeftVelocity > 1.0) {

            frontLeftVelocity = 1.0;

        } else if (frontLeftVelocity < -1.0) {

            frontLeftVelocity = -1.0;

        }

        if (frontRightVelocity > 1.0) {

            frontRightVelocity = 1.0;

        } else if (frontRightVelocity < -1.0) {

            frontRightVelocity = -1.0;

        }

        if (backLeftVelocity > 1.0) {

            backLeftVelocity = 1.0;

        } else if (backLeftVelocity < -1.0) {

            backLeftVelocity = -1.0;

        }

        if (backRightVelocity > 1.0) {

            backRightVelocity = 1.0;

        } else if (backRightVelocity < -1.0) {

            backRightVelocity = -1.0;

        }

        // Set velocities
        frontLeft.setVelocity(frontLeftVelocity * Qualifier2DriverOp.MAX_TICKS_PER_SECOND);
        frontRight.setVelocity(frontRightVelocity * Qualifier2DriverOp.MAX_TICKS_PER_SECOND);
        backLeft.setVelocity(backLeftVelocity * Qualifier2DriverOp.MAX_TICKS_PER_SECOND);
        backRight.setVelocity(backRightVelocity * Qualifier2DriverOp.MAX_TICKS_PER_SECOND);
    }
    
  
}


