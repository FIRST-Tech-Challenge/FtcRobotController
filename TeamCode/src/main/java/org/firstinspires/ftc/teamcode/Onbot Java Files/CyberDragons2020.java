package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

// For the IMU
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous
@Disabled
public class CyberDragons2020 extends LinearOpMode {


  // The orientation/position is based on a view of a person looking at
  // the robot from the top, shooter at front, intake at back
  // Right is where the wabble goal mechanism is 
  
  private DcMotorEx driveBR; // Back Right drive motor 
  private DcMotorEx driveFR; // Front Right drive motor 
  private DcMotorEx driveBL; // Back Left drive motor 
  private DcMotorEx driveFL; // Front Left drive motor

  // The two shooter motors   
  private DcMotorEx shooter1; // the faster ultra planetary gear motor

  // motors and servo for the conveyor belt and the noodles intake  
  private DcMotor conveyor;     // motor that runs the conveyor/inclined plane 
  private DcMotor noodle;       // servo that runs the noodle intake mechanism
                                // Note that this is a servo and not a motor 
                                
  private Servo liftgate1; // servo for gate on the conveyor  
  private Servo liftgate2;
  private Servo liftgate3;
  
  // motor and servo for the wobble/waffle 
  private DcMotor wafflemotor;
  private Servo waffle1;
  private Servo waffle2;

  // the different sensors
  // Two color sensors for the launch line 
  private ColorSensor launchLineRight;
  private ColorSensor launchLineLeft;
  
  // Two distance sensors on the left 
  private DistanceSensor distanceLeftFront;
  private DistanceSensor distanceLeftBack;
  
  // One distance sensor in the front 
  private DistanceSensor distanceFront;
  
  // One distance sensor at the right 
  private DistanceSensor distanceRight;
  // One distance sensor in the back 
  private DistanceSensor distanceBack;

  // variables for Vuforia and the zone detection 
  private VuforiaCurrentGame vuforiaUltimateGoal;
  private TfodCurrentGame tfodUltimateGoal;
  private int AZONE = 1; // DONT CHANGE: these are const values
  private int BZONE = 2;
  private int CZONE = 3;
  private int final_zone = AZONE; // using a default so it doesnt mess things up

  // Variables for the IMU 
  BNO055IMU               imu;
  Orientation             lastAngles = new Orientation();
  double                  globalAngle, power = .30, correction;

  // 
  long autoStartTime = 0 ; 

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    initializeRobot(); // intialize the complete robot including camera, imu, etc.
    //setupWabbleGoal() ; // set up the wabble goal position
    
    runTestCode() ;  // run any test code if required 
    sleep(1000) ;
    liftgate3.setPosition(1);
    telemetry.addData(">", "Press Play to start");
    telemetry.update();

    // Wait for start command from Driver Station.
    waitForStart();

    // use the start
    autoStartTime = System.currentTimeMillis() ;
    
    if (opModeIsActive()) {
      
      // First shoot the rings 
      shootRingsFromStart(0.9) ;
      
      // move forward and detect the number of rings in the stack  
      detectZone() ; // determine number of rings and the zone for dropoff       
      
      // While detecting zone, robot moves front a few inches 
      // After that, straighten it and then use IMU to go straight
      // irrespective of the zone 
      // In the case of Zone B and Zone C, take the rings along 
      
      // So first, make it parallel to the perimeter wall 
      // Use the IMU or a manual way 
      
      // Adjust manually
      driveFR.setPower(1.0);
      driveBR.setPower(1.0);
      driveFL.setPower(0.0);
      driveBL.setPower(0.0);
      sleep(150);
      stopAllWheelsPower();
      
      // move forward to the Launch Line
      // drop the wabble to save some time  
      //moveForwardToLaunchLineAndLowerWabble(1.0) ;
      
      
      // strafe to the right close to the perimeter wall and adjust 
      //strafeToPerimeterWallAndAdjust(1.0) ;
      
      // move along perimeter wall to launchLine  
      // Note that the "..WithoutAlign" functions uses the IMU  
      //moveForwardToLaunchLineWithoutAlign(1.0) ;
      
      // Alternate function to use 
      //moveForwardAlongPerimeterToLaunchLineAndAlign(1.0) ;
      
      // REST of the actions are different for different zones 

      if (final_zone == AZONE) {
        targetZoneA();
      } else {
        if (final_zone == BZONE) {
          targetZoneB(); 
        } else {
          targetZoneC();
        }
      }
      
      
      // sleep a bit so things settle down a bit 
      // strafe along the LaunchLine till we reach the shooting location
      //strafeRightToShootingPositionOnLaunchLine() ;
      //strafeToShootingPositionForHighGoal() ;
      //sleep(1000); 
      // shoot all 3 power shots, moving for each one  
      //shootPowerShots();
      //shootHighGoal();
      
      // Ensure that the robot is back on the launchLine
        
    } // end of if Opmode is active 
    
    // Deactivate TFOD.
    tfodUltimateGoal.deactivate();

    vuforiaUltimateGoal.close();
    tfodUltimateGoal.close();
    
  } // end of runOpmode

  // A simple run test code function   
  private void runTestCode() {
    /* execute any test code if required 
     this is run before the start is pressed and helpful for debugging
     */
     runShooter(0.9) ;
     sleep(1500);
     liftgate2.setPosition(1.0) ;
     sleep(1500);
     runInclinedPlane(1.0); 
     sleep(3500);
     runShooter(1.0) ;
     runNoodleIntake(0.4) ;
     sleep(40000) ;
     liftgate2.setPosition(0.5) ;
     stopNoodleIntake();
     stopInclinedPlane();
     stopShooter();
     
     /*
     liftgate3.setPosition(1);
     sleep(3000);
     
     liftgate3.setPosition(0);
     sleep(3000);
     
     liftgate3.setPosition(0.5);
     */
     
     //testSensors() ;
     //shootRingsFromStart(0.9);
     //testShooter1velocity();
     //shootRingsFromStart(0.9) ;
     /*
     runInclinedPlane(1.0);
     liftgate2.setPosition(1); 
     liftgate1.setPosition(1);
     sleep(10000);
     liftgate2.setPosition(0.5);
     liftgate1.setPosition(0.5);
     stopInclinedPlane();
     */
     /*
     shooter1.setPower(0.9);
     sleep(2000);
     conveyor.setPower(1);
     sleep(20000);
     shooter1.setPower(0);
     conveyor.setPower(0);
     */
     
     /*
     conveyor.setPower(1.0) ;
     noodle.setPower(1.0) ;
     sleep(30000) ;
     noodle.setPower(0.0) ;
     conveyor.setPower(0) ;
     */
     /*
     driveBL.setPower(1);
     sleep(3000);
     stopAllWheelsPower();
     
     driveFL.setPower(1);
     sleep(3000);
     stopAllWheelsPower();
     
     driveBR.setPower(1);
     sleep(3000);
     stopAllWheelsPower();
     
     driveFR.setPower(1);
     sleep(3000);
     stopAllWheelsPower();
     */
     
     /*
     strafeRightPower(1.0);
     sleep(4000);
     stopAllWheelsPower();
     
     sleep(100);
     
     strafeLeftPower(1.0);
     sleep(4000);
     stopAllWheelsPower();
     */
     
     
     //testIMU();
     //testForwardIMU() ; 
     //testBackwardIMU() ; 
     //testStrafeRightIMU() ; 
     //testStrafeLeftIMU() ; 
     
     //shootPowerShots() ;
     //testShooter1velocity() ;
     //runNoodleIntake(1.0);
     //sleep(15000);
     
    //strafeRightToShootingPositionOnLaunchLine() ;
    /* 
    runShooter(1.0);
    sleep(3000);
    
    runInclinedPlane(1.0);
    // assuming that the first ring is shot within 2 seconds
    // might need to use a sensor to check when the first one is being shot 
    shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    sleep(2000) ;
    double  sh_vel = 0.0 ; 
    sh_vel = shooter1.getVelocity() ;

    while (sh_vel > 2000) {
      sh_vel = shooter1.getVelocity() ;
    }
    
    telemetry.addData("Shooter1 velocity End", " " + sh_vel) ;
    telemetry.update() ;
    sleep(5000) ;
    
    stopInclinedPlane();
    stopShooter();
    */
     /*
    wafflemotor.setPower(-1);
    sleep(500);
    wafflemotor.setPower(0);
    */
    //testRightColorSensorRun() ;
    //moveBackwardToLaunchLineAndAlign(0.75) ; 
    //testRightColorSensorRun() ;
    //moveBackwardToLaunchLineAndAlign(0.75) ;
    //moveBackwardAlongPerimeterToLaunchLineAndAlign(0.75) ;
    //sleep(10000) ; // sleep for sometime so you can see the values
    
    //rotate(180, 0.75) ;
    
     /*
     detectZone();
     sleep(5000) ;
     
     strafeRightPower(1.0) ;
     while (distanceRight.getDistance(DistanceUnit.INCH) > 15) {
       ;
     }
     stopAllWheelsPower();
     moveBackwardCorrection(1.0);
     */
    //rotate(180, 0.5);
    
    //stopAllWheelsPower();
    
    return ;
  }
  
  private void setupWabbleGoal() {
    // initial position of the servos 
    // Note that this will cause the waffle goal grabbers to close 
    // and will start consuming power
    
    wafflemotor.setPower(-1);
    sleep(1500);
    wafflemotor.setPower(0);
    
    sleep(5000);
    
    
    waffle1.setPosition(1);
    waffle2.setPosition(-1);
    //liftgate.setPosition(1);
  }

  private void initializeRobot() {
    
      // Initialize the vuforia first, so the camera stream is visible
      initializeCamera();

      //hardware maps of drive motors
      driveBR = hardwareMap.get(DcMotorEx.class, "drive1");
      driveFR = hardwareMap.get(DcMotorEx.class, "drive2");
      driveFL = hardwareMap.get(DcMotorEx.class, "drive3");
      driveBL = hardwareMap.get(DcMotorEx.class, "drive4");
      
      // hardware map for shooter, intake and conveyor 
      shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
      conveyor = hardwareMap.dcMotor.get("intake");
      noodle = hardwareMap.get(DcMotor.class, "intakemotor");
      liftgate1 = hardwareMap.get(Servo.class, "liftgate1");
      liftgate2 = hardwareMap.get(Servo.class, "complianceWheels");
      liftgate3 = hardwareMap.get(Servo.class, "liftgate3");
      // hardware map for wobble/waffle 
      wafflemotor = hardwareMap.dcMotor.get("wafflemotor");
      waffle1 = hardwareMap.get(Servo.class, "waffle1");
      waffle2 = hardwareMap.get(Servo.class, "waffle2");
      
      // hardware map for sensors 
      launchLineLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
      launchLineRight = hardwareMap.get(ColorSensor.class, "colorRight");
      distanceLeftFront = hardwareMap.get(DistanceSensor.class, "distanceLeftFront");
      distanceLeftBack = hardwareMap.get(DistanceSensor.class, "distanceLeftBack");
      distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront" );
      distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight" );
      distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");

      //directions for the differnet motors 
      driveBR.setDirection(DcMotor.Direction.REVERSE);
      driveFR.setDirection(DcMotor.Direction.FORWARD);
      driveBL.setDirection(DcMotor.Direction.FORWARD);
      driveFL.setDirection(DcMotor.Direction.REVERSE);
      
      conveyor.setDirection(DcMotor.Direction.FORWARD); 
      noodle.setDirection(DcMotor.Direction.REVERSE);
      
      shooter1.setDirection(DcMotor.Direction.FORWARD);
      wafflemotor.setDirection(DcMotor.Direction.FORWARD);

      
      //zero power behavior of the different motors
      driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      // set the mode of operation; assumes that the encoder wires are connected
      driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      // set shooter1 and shooter2 to use the encoder 
      shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;

      initializeIMU();
      telemetry.addData("Status", "Initialized");
      telemetry.update();

  }
  
  private void initializeCamera() {
    
    vuforiaUltimateGoal = new VuforiaCurrentGame();
    tfodUltimateGoal = new TfodCurrentGame();

    // Sample TFOD Op Mode
    // Initialize Vuforia.
    /*
    vuforiaUltimateGoal.initialize(
        "", // vuforiaLicenseKey
        hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
        "", // webcamCalibrationFilename
        false, // useExtendedTracking
        false, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        0, // xAngle
        0, // yAngle
        0, // zAngle
        true); // useCompetitionFieldTargetLocations
    */
    // Set min confidence threshold to 0.7; VL: maybe higher confidence??
    tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.7F, true, true);

    // Initialize TFOD before waitForStart.
    // Init TFOD here so the object detection labels are visible
    // in the Camera Stream preview window on the Driver Station.
    tfodUltimateGoal.activate();

    // Enable following block to zoom in on target.
    // IMPORTANT: Camera zoom and view area are important parameters 
    tfodUltimateGoal.setZoom(2.5, 16 / 9);
    
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
    {   // turn right.
        leftPower = power;
        rightPower = -power;
    }
    else if (degrees > 0)
    {   // turn left.
        leftPower = -power;
        rightPower = power;
    }
    else return;

    // set power to rotate.
    driveFL.setPower(leftPower);
    driveBL.setPower(leftPower);
    driveFR.setPower(rightPower);
    driveBR.setPower(rightPower);
    
    // rotate until turn is completed.
    if (degrees < 0)
    {
        // On right turn we have to get off zero first.
        while (getAngle() == 0) {}

        while (getAngle() > degrees) {}
    }
    else    // left turn.
        while (getAngle() < degrees) {}

    // turn the motors off.
    stopAllWheelsPower();
    

    // reset angle tracking on new heading.
    resetAngle();
  }


  private void detectZone() {
    // Sometimes Vuforia does not detect well on first go. 
    // we may need to do a couple of tries to be really confident of the 
    // response from the video detection. 
    // For each attempt the robot moves closer to the rings a little bit
    
    // NOTE: Remove telemetry to save a bit of time
    
    int nvideo_attempts = 5; // change this if required. You can change it to 1 as well
    int azone_successes = 0 ;
    float azone_confidence = 0.0F ;
    int bzone_successes = 0 ;
    float bzone_confidence = 0.0F ;
    int czone_successes = 0 ;
    float czone_confidence = 0.0F ;

    List<Recognition> recognitions;
    int index = 0; 

    // get the robot moving and during movement it can check the detection  
    // a number of times 
    // Check below before changing the speed to higher than 0.5
    moveForwardPower(0.75);
    
//    while (nvideo_attempts > 0) {
    while (distanceBack.getDistance(DistanceUnit.INCH) < 24) {
      
      // do only the first nvideo attempts; otherwise it will get too 
      // close to the stack and the camera will not have them in view
      if (nvideo_attempts > 0 ) {
        // give it some time before checking for each attempt 
        sleep(100);
        
        nvideo_attempts-- ; // reduce count first so you dont forget later 
        
        // Put loop blocks here.
        // Get a list of recognitions from TFOD.
        recognitions = tfodUltimateGoal.getRecognitions();
  
        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (recognitions.size() == 0) {
          azone_successes++ ; 
          azone_confidence += 0.9F ; // just giving it something nominal rather than 1.0 
          telemetry.addData("TFOD ", "No items detected.");
          telemetry.addData("Target Zone ", "A");
        } else {
          if (recognitions.size() > 1) {
            telemetry.addData("TFOD ", "More than 1 item detected");
          }
          
          index = 0;
          // Iterate through list and call a function to
          // display info for each recognized object.
          for (Recognition recognition : recognitions) {
  
            if (recognition.getLabel().equals("Single")) {
              telemetry.addData("Target Zone ", "B");
              bzone_successes++ ; 
              bzone_confidence += recognition.getConfidence() ;
            } else if (recognition.getLabel().equals("Quad")) {
              telemetry.addData("Target Zone ", "C");
              czone_successes++ ; 
              czone_confidence += recognition.getConfidence() ;
            } else {
              telemetry.addData("TFOD ", "Unknown label " + recognition.getLabel());
              telemetry.addData("Target Zone ", "Unknown");
            }
            // Display info.
            //displayInfo(index); Commenting this out because details are not required
            // Increment index.
            index = index + 1;
          } // end of recoginition list 
        } // end of else for recognition list empty
        //telemetry.update() ; // make sure telemetry is updated 
        
      } // end of video attempts
      else {
        break ;
      }
    } // exit when distance is greater than a certain value  
    
    stopAllWheelsPower();

    // Remove all telemetry at some time to increase response time 
    
    telemetry.addData("AZone ", azone_successes + " " + azone_confidence);
    telemetry.addData("BZone ", bzone_successes + " " + bzone_confidence);
    telemetry.addData("CZone ", czone_successes + " " + czone_confidence);
    
    // Now decide on the final zone 
    // This can be done in a number of ways 
    // simple way would be to use the accumulated confidence levels 
    if (azone_confidence > bzone_confidence) {
      if (azone_confidence > czone_confidence) {
        // A is the clear winner
        final_zone = AZONE ; 
        telemetry.addData("Final Zone", " A");
      } else {
        // C is the clear winner
        final_zone = CZONE ;
        telemetry.addData("Final Zone", " C");
      }
    } else {
      if (bzone_confidence > czone_confidence){
        // B is the clear winner 
        final_zone = BZONE ;
        telemetry.addData("Final Zone", " B");
      } else {
        // C is the winner
        final_zone = CZONE ;
        telemetry.addData("Final Zone", " C");
      }
    }
    telemetry.update(); 
    
    // by the end of this routine, final_zone carries the zone value 
  }
  
  private void moveForwardPower(double power) {
    driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveBR.setPower(power);
    driveFR.setPower(power);
    driveBL.setPower(power);
    driveFL.setPower(power);
  }
  
  private void moveForwardVelocity(double nticks) {
    driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveBL.setVelocity(nticks) ;
    driveFL.setVelocity(nticks) ;
    driveBR.setVelocity(nticks) ;
    driveFR.setVelocity(nticks) ;
  }


  private void moveBackwardPower(double power) {
    driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveBR.setPower(power * -1.0);
    driveFR.setPower(power * -1.0);
    driveBL.setPower(power * -1.0);
    driveFL.setPower(power * -1.0);
  }

  private void moveBackwardVelocity(double nticks) {
    driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveBL.setVelocity(nticks * -1.0) ;
    driveFL.setVelocity(nticks * -1.0) ;
    driveBR.setVelocity(nticks * -1.0) ;
    driveFR.setVelocity(nticks * -1.0) ;
  }
  
  private void moveForwardIMU(double power) {
    
    correction = checkDirection();

    driveBL.setPower(power - correction);
    driveFL.setPower(power - correction);
    driveBR.setPower(power + correction);
    driveFR.setPower(power + correction);

  }
  
  private void moveBackwardIMU(double power) {
    
    correction = checkDirection();

    driveBL.setPower((power - correction) * -1.0);
    driveFL.setPower((power - correction) * -1.0);
    driveBR.setPower((power + correction) * -1.0);
    driveFR.setPower((power + correction) * -1.0);

  }
  
  private void strafeRightIMU(double power) {
    
    correction = checkDirection();

    driveBL.setPower((power - correction));
    driveFL.setPower((power - correction) * -1.0);
    driveBR.setPower((power + correction) * -1.0);
    driveFR.setPower((power + correction));

  }

  private void strafeLeftIMU(double power) {
    
    correction = checkDirection();

    driveBL.setPower((power + correction) * -1.0);
    driveFL.setPower((power + correction));
    driveBR.setPower((power - correction));
    driveFR.setPower((power - correction) * -1.0);

  }
  
  private void strafeRightPower(double power) {
    driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveBR.setPower(power * -1.0);
    driveFR.setPower(power);
    driveBL.setPower(power);
    driveFL.setPower(power * -1.0);
  }
  
  private void strafeRightVelocity(double nticks) {
    driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveBR.setVelocity(nticks * -1.0) ;
    driveFR.setVelocity(nticks) ;
    driveBL.setVelocity(nticks) ;
    driveFL.setVelocity(nticks * -1.0) ;
  }
  
  private void strafeLeftPower(double power) {
    driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    driveBR.setPower(power);
    driveFR.setPower(power * -1.0);
    driveBL.setPower(power * -1.0);
    driveFL.setPower(power);
  }

  private void strafeLeftVelocity(double nticks) {
    driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    driveBR.setVelocity(nticks) ;
    driveFR.setVelocity(nticks * -1.0) ;
    driveBL.setVelocity(nticks * -1.0) ;
    driveFL.setVelocity(nticks) ;
  }
  
  
  
  private void moveForwardAlongPerimeterToLaunchLineAndAlign(double plevel) {
    
    double target_distance = 2.0; // 2 parameters for tuning 
    double max_deviation = 1.0 ;   // Change these two parameters for alignment 
    double cur_distance = 0.0;
    double abs_distance;
    double power_correction = 0.0;
    
    int minWhiteColorAlpha = 500 ; // Another parameter to tune 
    int colorRight = 0;
    int colorLeft = 0;
    
    // first just make the plevel negative
    moveForwardPower(plevel);    
    
    colorRight = launchLineRight.alpha() ;
    colorLeft = launchLineLeft.alpha() ;
    // Do the while loop until a certain time is reached 
    while ((colorRight < minWhiteColorAlpha) && (colorLeft < minWhiteColorAlpha)) {
      
      cur_distance = distanceRight.getDistance(DistanceUnit.INCH); 
      abs_distance = Math.abs(cur_distance - target_distance);
      
      if (abs_distance >= max_deviation) {
        
        if (cur_distance >= (target_distance + max_deviation)) {
          strafeLeftPower(1.0) ;
          while(distanceLeftFront.getDistance(DistanceUnit.INCH) > target_distance) {
          }
          moveForwardPower(plevel) ;
        } else {
          if (cur_distance <= (target_distance - max_deviation)) {
            strafeRightPower(1.0) ;
            while(distanceLeftFront.getDistance(DistanceUnit.INCH) < target_distance) {
            }
            moveForwardPower(plevel) ;
          } else {
            // it should not come here 
            moveForwardPower(plevel) ;
            // Look at exception handling later ; for now just use telemetry 
            telemetry.addData("Exception", "Error in target distance") ;
            telemetry.update() ;
            //throw new Exception("Error in target distance") ;
            // Exception requires try and catch blocks 
          }
        }
      }

      // check if we are on the launch line
      colorRight = launchLineRight.alpha() ;
      colorLeft = launchLineLeft.alpha() ;
      
    }
    //stopAllWheelsPower();
    //telemetry.addData("Color sensor First","Right: " + colorRight + " Left: " + colorLeft) ;
    //telemetry.addData("Color sensor Second","Right: " + launchLineRight.alpha() + " Left: " + launchLineLeft.alpha()) ;
    //telemetry.update();
    //sleep(1000);
    
    // Now align it to the LaunchLine using the two color sensors 
    if ((colorRight >= minWhiteColorAlpha) && (colorLeft >= minWhiteColorAlpha)) {
        // dont need to do anything 
    } else {
      if ((colorRight < minWhiteColorAlpha) && (colorLeft >= minWhiteColorAlpha)) {
        // move the right wheels 
        // Note that we are moving backward 
        // May need to go a bit slower rather than full speed
        driveFR.setPower(plevel);
        driveBR.setPower(plevel);
        driveFL.setPower(0.0);
        driveBL.setPower(0.0);
        // Note that the left wheels are stationary 
        while (launchLineRight.alpha() < minWhiteColorAlpha) {
          // it might be required to move both 
          // How about getting rightColor closer to leftColor 
          // Will it be any better? 
          ;
        }
        stopAllWheelsPower();
      } else {
        if ((colorRight >= minWhiteColorAlpha) && (colorLeft < minWhiteColorAlpha)) {
          // move the left wheels 
          driveFL.setPower(plevel);
          driveBL.setPower(plevel);
          driveFR.setPower(0.0);
          driveBR.setPower(0.0);
          while (launchLineLeft.alpha() < minWhiteColorAlpha) {
            // it might be required to move both 
            // How about getting rightColor closer to leftColor 
            // Will it be any better? 
            ;
          }
          stopAllWheelsPower();
        } else {
          // this means that neither of the two sensors are on the whiteline
          // Ideally, it should not come here. 
          // If it does, throw an exception 
          // Look at exception handling later ; for now just use telemetry 
          telemetry.addData("Exception", "Both color sensors shows less than minWhiteColorAlpha 1") ;
          telemetry.update() ;
          //throw new Exception("Both color sensors shows less than minWhiteColorAlpha") ;
          // Exception requires try and catch blocks 
        }
      }
    }
    stopAllWheelsPower();
  }


  private void moveBackwardToLaunchLineAndAlign(double plevel) {
    
    int minWhiteColorAlpha = 500 ; // Another parameter to tune 
    int colorRight = 0;
    int colorLeft = 0;
    
    int colorRightAfterBreak ;
    
    // start moving back 
    resetAngle();

    //moveBackwardPower(plevel);    
    
    // Do the while loop until the color is reached
    colorRight = launchLineRight.alpha() ;
    colorLeft = launchLineLeft.alpha() ;
    while ((colorRight < minWhiteColorAlpha) && (colorLeft < minWhiteColorAlpha)) {
      colorRight = launchLineRight.alpha() ;
      colorLeft = launchLineLeft.alpha() ;
      moveForwardIMU(-1.0 * plevel) ; // NOTE
    }
    //stopAllWheelsPower();
    //sleep(2000);
    //telemetry.addData("Color sensor First","Right: " + colorRight + " Left: " + colorLeft) ;
    //telemetry.addData("Color sensor Second","Right: " + launchLineRight.alpha() + " Left: " + launchLineLeft.alpha()) ;
    //telemetry.update();
    //sleep(1000);
    
    // check if the braking action caused some changes 
    
    
    // Now align it to the LaunchLine using the two color sensors 
    if ((colorRight >= minWhiteColorAlpha) && (colorLeft >= minWhiteColorAlpha)) {
        // dont need to do anything 
    } else {
      if ((colorRight < minWhiteColorAlpha) && (colorLeft >= minWhiteColorAlpha)) {
        // move the right wheels 
        // Note that we are moving backward 
        // May need to go a bit slower rather than full speed
        driveFR.setPower(plevel * -1.0);
        driveBR.setPower(plevel * -1.0);
        driveFL.setPower(0.0);
        driveBL.setPower(0.0);
        // Note that the left wheels are stationary 
        while (launchLineRight.alpha() < minWhiteColorAlpha) {
          // it might be required to move both 
          // How about getting rightColor closer to leftColor 
          // Will it be any better? 
          ;
        }
        stopAllWheelsPower();
      } else {
        if ((colorRight >= minWhiteColorAlpha) && (colorLeft < minWhiteColorAlpha)) {
          // move the left wheels 
          driveFL.setPower(plevel * -1.0);
          driveBL.setPower(plevel * -1.0);
          driveFR.setPower(0.0);
          driveBR.setPower(0.0);
          while (launchLineLeft.alpha() < minWhiteColorAlpha) {
            // it might be required to move both 
            // How about getting rightColor closer to leftColor 
            // Will it be any better? 
            ;
          }
          stopAllWheelsPower();
        } else {
          // this means that neither of the two sensors are on the whiteline
          // Ideally, it should not come here. 
          // If it does, throw an exception 
          // Look at exception handling later ; for now just use telemetry 
          telemetry.addData("Exception", "Both color sensors shows less than minWhiteColorAlpha 2") ;
          telemetry.update() ;
          //throw new Exception("Both color sensors shows less than minWhiteColorAlpha") ;
          // Exception requires try and catch blocks 
        }
      }
    }
    stopAllWheelsPower();
  }
  
  private void strafeLeftToLaunchLine(double plevel) {
  
    int minWhiteColorAlpha = 500 ; // Another parameter to tune 
    int colorRight = 0;
    int colorLeft = 0;
    
    int colorRightAfterBreak ;
    
    // start moving back 
    resetAngle();
  
    //moveBackwardPower(plevel);    
    
    // Do the while loop until the color is reached
    colorRight = launchLineRight.alpha() ;
    colorLeft = launchLineLeft.alpha() ;
    while ((colorRight < minWhiteColorAlpha) && (colorLeft < minWhiteColorAlpha)) {
      colorRight = launchLineRight.alpha() ;
      colorLeft = launchLineLeft.alpha() ;
      strafeRightIMU(-1.0 * plevel) ; // NOTE
    }
    stopAllWheelsPower();
  }

  private void moveForwardToLaunchLineWithoutAlign(double plevel) {
    
    int minWhiteColorAlpha = 500 ; // Another parameter to tune 
    int colorRight = 0;
    int colorLeft = 0;
    
    int colorRightAfterBreak ;
    
    // start moving back 
    resetAngle();

    //moveBackwardPower(plevel);    
    
    // Do the while loop until the color is reached
    colorRight = launchLineRight.alpha() ;
    colorLeft = launchLineLeft.alpha() ;
    while ((colorRight < minWhiteColorAlpha) && (colorLeft < minWhiteColorAlpha)) {
      colorRight = launchLineRight.alpha() ;
      colorLeft = launchLineLeft.alpha() ;
      moveForwardIMU(plevel) ; // NOTE
    }
    stopAllWheelsPower();
  }
  
  private void moveForwardToLaunchLine(double plevel) {
    
    int minWhiteColorAlpha = 500 ; // Another parameter to tune 
    int colorRight = 0;
    int colorLeft = 0;
    
    // start moving back 
    resetAngle();

    // Do the while loop until the color is reached
    colorRight = launchLineRight.alpha() ;
    colorLeft = launchLineLeft.alpha() ;
    while ((colorRight < minWhiteColorAlpha) && (colorLeft < minWhiteColorAlpha)) {
      colorRight = launchLineRight.alpha() ;
      colorLeft = launchLineLeft.alpha() ;
      moveForwardIMU(plevel) ; // NOTE 
    }
    stopAllWheelsPower();
  }
  
  private void moveForwardToLaunchLineAndLowerWabble(double plevel) {
    
    int minWhiteColorAlpha = 500 ; // Another parameter to tune 
    int colorRight = 0;
    int colorLeft = 0;
    
    int colorRightAfterBreak ;
    
    // start moving back 
    resetAngle();

    // start a timer for lowering the wabble only    
    long tempTime = 0; 
    long lowerTime = 800 ; // tune this parameter as required 
    // VERY IMPORTANT: This should be lower than the time it takes to get 
    // to the launch line 
    boolean startedWabbleMotor = false ; 
    boolean stoppedWabbleMotor = true ; 
    
    tempTime = System.currentTimeMillis() ; 

    // Do the while loop until the color is reached
    colorRight = launchLineRight.alpha() ;
    colorLeft = launchLineLeft.alpha() ;
    while ((colorRight < minWhiteColorAlpha) && (colorLeft < minWhiteColorAlpha)) {
      colorRight = launchLineRight.alpha() ;
      colorLeft = launchLineLeft.alpha() ;
      moveForwardIMU(plevel) ; // NOTE 
      
      // start lowering wabble goal 500 seconds into the run 
      if(startedWabbleMotor == false) {
        if ((System.currentTimeMillis() - tempTime) < 500) { 
          wafflemotor.setPower(1); // starting to lower the wabble goal
          startedWabbleMotor = true ;
          stoppedWabbleMotor = false ;
          tempTime = System.currentTimeMillis() ; // reset the start time 
        }
      }
      // check if the wabble motor has already been stopped  
      if (stoppedWabbleMotor == false) {
        if ((System.currentTimeMillis() - tempTime) > lowerTime) {
          wafflemotor.setPower(0) ;
          stoppedWabbleMotor = true ;
        }
      }
    }
    stopAllWheelsPower();
    // Just to be sure, stop the wabble motor as well 
    wafflemotor.setPower(0) ;
    
  }
  
  // do we have to set the mode when switching off?  
  public boolean shouldStop() {
    return (distanceRight.getDistance(DistanceUnit.INCH) > 7);
  }
      
  private void stopAllWheelsPower() {
    driveBL.setPower(0.0);
    driveFL.setPower(0.0);
    driveBR.setPower(0.0);
    driveFR.setPower(0.0);
  }

  private void runShooter(double power) {
    shooter1.setPower(power);
  }
  private void runLiftGate2() {
    
  }
  
  private void stopShooter() {
    shooter1.setPower(0.0);
  }

  private void runInclinedPlane(double power) {
      conveyor.setPower(power);
  }
  
  private void stopInclinedPlane() {
      conveyor.setPower(0.0);
  }

  private void runNoodleIntake(double power) {
      noodle.setPower(power);
  }
  
  private void stopNoodleIntake() {
      noodle.setPower(0.0);
  }

  private void dropWaffleGoal() {
    
      // lower the waffle motor and then stop it 
      wafflemotor.setPower(1);
      sleep(1500); // this time can be saved by lowering the wabble goal before 
      wafflemotor.setPower(0);
      
      // Open the grips 
      waffle1.setPosition(-1);
      waffle2.setPosition(1);

  }
  

  private void targetZoneA() {
    
    // before dropping the wabble goal rotate 90 
    // the lowered intake mechanism may hit the perimeter wall 
    // so we  may need to go a bit forward and rotate a bit more than 90
  
    moveForwardToLaunchLineAndLowerWabble(1.0) ;
    
    // moving forward a bit 
    moveForwardPower(1.0) ;
    sleep(500) ;
    stopAllWheelsPower();
    
    // rotating 
    rotate(90,1.0) ;
    
    // move forward 
    moveForwardPower(1.0) ;
    sleep(1000) ;
    stopAllWheelsPower();
    

    /*    
    // move Backward till the robot is a few inches from the perimeter wall 
    // Use the back distance sensor 
    resetAngle();
    while (distanceBack.getDistance(DistanceUnit.INCH) > 10) {
      // Note that what is being detected here is the perimeter wall
      // and not the wabble goal itself  
      moveForwardIMU(-1.0 ) ; // NOTE
    }
    stopAllWheelsPower();
    */
    // move the hold on the wobble goal.
    // Open the grips 
    waffle1.setPosition(-1);
    waffle2.setPosition(1);
    
    /*
    // strafe left to clear off the waffle 
    strafeLeftPower(1.0) ;
    sleep(250) ;
    stopAllWheelsPower();
    */
    // If target is to just get one wabble goal then get to the launch line and stop 
    
    // Comment out the rest of the code below 
    // But if we are ambitious, get ready to pick up the second wabble 
    
    // First move forward until robot is some distance from the perimeter   
    /*
    moveForwardPower (1.0) ;
    while (distanceBack.getDistance(DistanceUnit.INCH) < 25) ; // change parameter 
    stopAllWheelsPower();
    
    // Now strafe right to get to the second wabble 
    resetAngle();
    
    while (distanceRight.getDistance(DistanceUnit.INCH) > 12) {
      // Note that what is being detected here is the perimeter wall
      // and not the wabble goal itself  
      strafeRightPower(1.0) ; // NOTE
    }
    strafeRightPower(1.0) ;
    sleep(500);
    stopAllWheelsPower();
    */

    
    /*
    // now  rotate a bit so the wabble grabbers are in line with the wablle goal    
    rotate (-45,1.0) ; // angle needs to be changed ; make sure it is not over-rotated 
    
    // now continue to strafe right a bit more
    // Ideally the distanceRight sensor can be used to see if we are sufficently 
    // close to the wabble goal, but for now we are doing something manual  
    strafeRightPower(1.0) ;
    sleep (500) ;
    */
    
    /*
    // hold the wabble goal 
    waffle1.setPosition(1);
    waffle2.setPosition(-1);
    sleep (2000) ;  // lets the grippers close properly 
    
    // lift the wabble goal slightly so it doesnt drag on the floor 
    wafflemotor.setPower(-1);
    sleep(250); // this time can be saved by lowering the wabble goal before 
    wafflemotor.setPower(0);


    // The below is for straffing to the launch line    
    // Now strafe left all the way left to the launch line 
    int minWhiteColorAlpha = 500 ; // Another parameter to tune 
    int colorRight = 0;
    int colorLeft = 0;
    
    strafeLeftPower(1.0) ;
    // Do the while loop until the color is reached
    colorRight = launchLineRight.alpha() ;
    colorLeft = launchLineLeft.alpha() ;
    while ((colorRight < minWhiteColorAlpha) && (colorLeft < minWhiteColorAlpha)) {
      colorRight = launchLineRight.alpha() ;
      colorLeft = launchLineLeft.alpha() ;
    }
    stopAllWheelsPower();

    //strafeLeftToLaunchLine(1.0) ;
    
    // we may need to move back some more a bit so adjust accordingly 
    strafeLeftPower(1.0) ;
    sleep(500) ;
    stopAllWheelsPower();
    
    rotate (-90,1.0) ;
    
    moveForwardPower(1.0) ;
    sleep(500) ;
    stopAllWheelsPower();
    */
    /*
    // move back a bit or until the back sensor is a certain distance from the wall  
    moveBackwardPower(1.0) ;
    while (distanceBack.getDistance(DistanceUnit.INCH) > 15) ;
    stopAllWheelsPower();
    */
    
    /*
    // drop the wabble goal 
    wafflemotor.setPower(1);
    sleep(250); // this time can be saved by lowering the wabble goal before 
    wafflemotor.setPower(0);
    
    // Open the grips 
    waffle1.setPosition(-1);
    waffle2.setPosition(1);
    
    // hopefully we are on the launch line 
    // if not get there.
    
    // Phew!! 
    */
  }

  private void targetZoneB() {

    moveForwardToLaunchLineAndLowerWabble(1.0) ;

    // On reaching the launch line 
    // Moving backward to clear from the ring
    moveBackwardPower(1.0);
    sleep(250);
    stopAllWheelsPower();

    // First, rotate 90 degrees 
    rotate(50, 1.0) ;
    
    // may need to move it forward some more 
    moveForwardPower(1.0) ;
    sleep(750) ;
    stopAllWheelsPower();

    // strafe right to get to Zone B     
    strafeRightPower(1.0) ;
    sleep(1500) ;
    stopAllWheelsPower();
    
    moveBackwardPower(1.0) ;
    sleep(1000) ;
    stopAllWheelsPower();
    
    // Open the grips 
    waffle1.setPosition(-1);
    waffle2.setPosition(1);
    
    // strafe left and get back to the Launch line  
    strafeLeftPower(1.0) ;
    sleep(150) ;
    stopAllWheelsPower();

    moveForwardPower(1.0) ;
    sleep(1000) ;
    stopAllWheelsPower();
    // run noodle earlier so it touches the floor 
    runNoodleIntake(1.0) ;
    
    strafeLeftPower(1.0) ;
    sleep(500) ;
    stopAllWheelsPower();
    // run the inclined plane and the noodles 
    // first ensure the liftgate3 
    liftgate3.setPosition(0.5) ; // ensure that the rings are not shot 
    runInclinedPlane(1.0) ;
    
    // Now move it back; hopefully it will pick it up the rings on the way 
    moveBackwardPower(1.0) ;
    sleep(1000) ;
    stopAllWheelsPower();
    
    // temporarily stop the inclined plane 
    stopInclinedPlane();
    
    // rotate and get ready for high Goal 
    rotate(-60,1.0) ;
    
    // First get the shooter reved up 
    // Run it at the right power level so as to target the high goal
    runShooter(1.0);
    moveBackwardPower(1.0);
    sleep(1000) ;
    stopAllWheelsPower();
    // you can use the velocity to determine if it has revved up sufficiently 
    // but for now just sleep for some time  
    sleep(1000);  //warming up shooter
    
    // Start running the inclined plane 
    liftgate3.setPosition(1.0) ; // ensure that the rings are not shot 
    runInclinedPlane(1.0);
    // For the second one, run the noodle so that it can push the third ring a bit 
    // and nudge the second ring to get going on the inclined plane
    runNoodleIntake(0.5); // note the power level   
    
    // sometimes the first ring is not getting shot so move the lift gate 
    liftgate1.setPosition(1.0);
    liftgate2.setPosition(1.0);

    double cur_time = System.currentTimeMillis() ;
    
    while ((System.currentTimeMillis() - autoStartTime) < 29000) ;
      
    moveForwardPower(1.0) ;  
    sleep(500) ;
    stopAllWheelsPower();
  }

  private void targetZoneC() {
    
    moveForwardToLaunchLine(1.0) ;
    
    // On reaching the launch line 
    // Moving backward to clear from the ring
    moveBackwardPower(1.0);
    sleep(500);
    stopAllWheelsPower();

    // First, rotate 90 degrees 
    rotate(60, 1.0) ;
    
    // may need to move it forward some more 
    moveForwardPower(1.0) ;
    sleep(750) ;
    stopAllWheelsPower();

    // strafe right to get to Zone C     
    strafeRightPower(1.0) ;
    wafflemotor.setPower(1.0) ;
    sleep(1000) ;
    wafflemotor.setPower(0.0) ;
    sleep(2500) ;
    stopAllWheelsPower();
    
    // Open the grips 
    waffle1.setPosition(-1);
    waffle2.setPosition(1);
    
    // strafe left and get back to the Launch line
    // could use the color sensor to detect the launch line  
    strafeLeftPower(1.0) ;
    
    // get to the launch line and use the colorLeft 
    int minWhiteColorAlpha = 500 ; // Another parameter to tune 
    int colorLeft = 0;
    
    // Do the while loop until the color is reached
    colorLeft = launchLineLeft.alpha() ;
    while (colorLeft < minWhiteColorAlpha) {
      colorLeft = launchLineLeft.alpha() ;
    }
    stopAllWheelsPower();
    
    // run noodle earlier so it touches the floor 
    runNoodleIntake(1.0) ;
    liftgate3.setPosition(0.5) ; // ensure that the rings are not shot 
    runInclinedPlane(1.0) ;
    
    // Now move it back; hopefully it will pick it up the rings on the way 
    moveBackwardPower(1.0) ;
    sleep(2000) ;
    stopAllWheelsPower();
    moveForwardPower(1.0) ;
    sleep(1000) ;
    stopAllWheelsPower();
    
    // temporarily stop the inclined plane 
    stopInclinedPlane();
    
    // rotate and get ready for high Goal 
    rotate(-90,1.0) ;
    
    // First get the shooter reved up 
    // Run it at the right power level so as to target the high goal
    runShooter(1.0);
    moveBackwardPower(1.0) ;
    sleep(1000) ;
    stopAllWheelsPower();
    sleep(1000) ; // revving the shooter for the remaining 1 second
    // you can use the velocity to determine if it has revved up sufficiently 
    // but for now just sleep for some time  
    
    // Start running the inclined plane 
    liftgate3.setPosition(1.0) ; // ensure that the rings are not shot 
    runInclinedPlane(1.0);
    // For the second one, run the noodle so that it can push the third ring a bit 
    // and nudge the second ring to get going on the inclined plane
    runNoodleIntake(0.5); // note the power level   
    
    // sometimes the first ring is not getting shot so move the lift gate 
    liftgate1.setPosition(1.0);
    liftgate2.setPosition(1.0);

    double cur_time = System.currentTimeMillis() ;
    
    while ((System.currentTimeMillis() - autoStartTime) < 29000) ;
      
    moveForwardPower(1.0) ;  
    sleep(500) ;
    stopAllWheelsPower();
  }
  
  private void strafeRightToShootingPositionOnLaunchLine() {
  
    // Assumption is that the robot is already on the LauchLine 
    runShooter(1.0);
    //sleep(3000);
    
    // Use the right strafing method
    
    strafeRightVelocity(400.0) ;
    
    while(distanceRight.getDistance(DistanceUnit.INCH) > 30) {
      // check for the color sensors and ensure that they are on the LaunchLine
      ;
    }
    
    stopAllWheelsPower();
    
  }

  private void strafeToShootingPositionForHighGoal() {
  
    // Assumption is that the robot is already on the LauchLine 
    runShooter(1.0);
    //sleep(3000);

    if(distanceLeftFront.getDistance(DistanceUnit.INCH) > 35) {
      
      strafeLeftVelocity(400) ;
      while(distanceLeftFront.getDistance(DistanceUnit.INCH) > 35) {
        // check for the color sensors and ensure that they are on the LaunchLine
        ;
      }

      // move back slightly
      moveBackwardPower(1.0) ;
      sleep(250) ;
      
      // currently adjustment only for B zone 
      if (final_zone == BZONE) {
        driveFL.setPower(-1.0);
        driveBL.setPower(-1.0);
        driveFR.setPower(0.0);
        driveBR.setPower(0.0);
        sleep(250) ;
      }
      stopAllWheelsPower();
      
    } else {
      
      strafeRightVelocity(400) ;
      while(distanceLeftFront.getDistance(DistanceUnit.INCH) < 35) {
        // check for the color sensors and ensure that they are on the LaunchLine
        ;
      }
      
      // move back slightly
      moveBackwardPower(1.0) ;
      sleep(250) ;
      
      driveFL.setPower(0.0);
      driveBL.setPower(0.0);
      driveFR.setPower(-1.0);
      driveBR.setPower(-1.0);
      sleep(200) ;
      stopAllWheelsPower();
    }
    
  }
  
  private void shootPowerShots() {
    
    // moved the shooter to earlier
    /*
    runShooter(0.8);
    sleep(3000);
    */
    runInclinedPlane(1.0);
    
    shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    double  sh_vel = 0.0 ;
    double cur_vel = 0.0 ;
    double target_vel = 0.0 ;
    sh_vel = shooter1.getVelocity() ;
    
    target_vel = sh_vel - 100 ;
    //telemetry.addData("Shooter velocity 1", " " + sh_vel) ;
    cur_vel = sh_vel ;
    while (cur_vel > target_vel) {
      cur_vel = shooter1.getVelocity() ;
    }
    sh_vel = cur_vel ; 
    //telemetry.addData("Shooter velocity 2", " " + sh_vel) ;
    
    // Allow it time to shoot.. how much? 
    sleep(500) ;
    
    //  turn slightly for the second powershot    
    driveFL.setPower(1.0);
    driveBL.setPower(1.0);
    driveFR.setPower(-1.0);
    driveBR.setPower(-1.0);
    sleep (250) ;
    stopAllWheelsPower();
    
    sh_vel = shooter1.getVelocity() ;
    target_vel = sh_vel - 100 ;
    //telemetry.addData("Shooter velocity 3", " " + sh_vel) ;
    cur_vel = sh_vel ;
    while (cur_vel > target_vel) {
      cur_vel = shooter1.getVelocity() ;
    }
    sh_vel = cur_vel ; 
    //telemetry.addData("Shooter velocity 4", " " + sh_vel) ;
    
    // Allow it time to shoot
    sleep(500) ;
    
    //  turn slightly for the third powershot   
    driveFL.setPower(1.0);
    driveBL.setPower(1.0);
    driveFR.setPower(-1.0);
    driveBR.setPower(-1.0);
    sleep (500) ;
    stopAllWheelsPower();
    
    sh_vel = shooter1.getVelocity() ;
    target_vel = sh_vel - 100 ;
    //telemetry.addData("Shooter velocity 5", " " + sh_vel) ;
    cur_vel = sh_vel ;
    while (cur_vel > target_vel) {
      cur_vel = shooter1.getVelocity() ;
    }
    sh_vel = cur_vel ; 
    //telemetry.addData("Shooter velocity 6", " " + sh_vel) ;
    
    // Allow it time to shoot
    sleep(2000) ;
    
    stopInclinedPlane();
    stopShooter();
    
  }
  
  private void shootHighGoal() {
    // run inclined plane 
    runInclinedPlane(1.0);
    //sleep (1000) ;
    
    long cur_time = System.currentTimeMillis() ;
    int noodle_state = 0 ;
    // At 29 seconds, make sure you have enough time to get to the launch line 
    while ((System.currentTimeMillis() - autoStartTime) < 29000) {
      
      if (noodle_state == 0) {
        if ((System.currentTimeMillis() - cur_time) > 2000) {
          noodle_state = 1 ;
          runNoodleIntake(0.25) ;
        }
      }
    }
    
    moveForwardPower(1.0) ;
    sleep(500) ;
    stopAllWheelsPower();
    
  }
  
  private void testRightColorSensorRun() {
    int redval = 0 ;
    int greenval = 0 ;
    int blueval = 0 ; 
    int combval = 0;
    int alphaval = 0 ;
    //double startTime = 0; 
    //double curTime = 0; 
    long startTime = 0; 
    
    //startTime = rtime.milliseconds() ; 
    startTime = System.currentTimeMillis() ; 

    redval = launchLineRight.red() ;
    greenval = launchLineRight.green() ;
    blueval = launchLineRight.blue() ; 
    alphaval = launchLineRight.alpha() ;
    telemetry.addData("Color Sensor Start", "red: " + redval + " green: " + greenval + " blue: " + blueval + " alpha: " + alphaval);

    moveBackwardPower(0.75);    
    
    while ((System.currentTimeMillis() - startTime) < 4000) {
      //telemetry.addData("distanceRight", " " + distanceRight.getDistance(DistanceUnit.INCH) + "in.");
      //telemetry.addData("distanceLeft", " " + distanceLeft.getDistance(DistanceUnit.INCH) + "in.");
      //telemetry.addData("distanceBack", " " + distanceBack.getDistance(DistanceUnit.INCH) + "in.");
      int tempredval = launchLineRight.red() ;
      int tempgreenval = launchLineRight.green() ;
      int tempblueval = launchLineRight.blue() ;
      int tempalphaval = launchLineRight.alpha() ;
      int tempval = tempredval + tempgreenval + tempblueval ; 
      if (tempval > combval) {
        redval = tempredval ; 
        greenval = tempgreenval ; 
        blueval = tempblueval ; 
        alphaval = tempalphaval ; 
        combval = tempval ; 
        
      }
      /*
      if (tempalphaval > 1000) {
        redval = tempredval ; 
        greenval = tempgreenval ; 
        blueval = tempblueval ; 
        alphaval = tempalphaval ; 
        combval = tempval ; 
        break ;
      }
      */
    }
    telemetry.addData("Color Sensor End", "red: " + redval + " green: " + greenval + " blue: " + blueval + " alpha: " + alphaval);
    telemetry.update() ;
    stopAllWheelsPower();
    
  }
  
  
  private void testStrafing(double plevel) {
    strafeRightPower(1.0) ;
    sleep(3000);
    stopAllWheelsPower();
    sleep(1000);
    
    strafeLeftPower(1.0) ;
    sleep(3000);
    stopAllWheelsPower();
    sleep(1000);
  }
  /*
  
  private void testShooter2velocity() {
    
    runShooter(1.0);
    sleep(3000);
    
    runInclinedPlane(1.0);
    // assuming that the first ring is shot within 2 seconds
    // might need to use a sensor to check when the first one is being shot 
    shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    sleep(2000) ;
    double  sh_vel = 0.0 ; 
    sh_vel = shooter2.getVelocity() ;

    long startTime = 0; 
    double cur_vel = 0.0 ; 
    
    //startTime = rtime.milliseconds() ; 
    startTime = System.currentTimeMillis() ; 

    telemetry.addData("Shooter2 velocity Start", " " + sh_vel) ;
    
    cur_vel = sh_vel ; 

    while ((System.currentTimeMillis() - startTime) < 4000) {
      // find out how the velocity reduces 
      sh_vel = shooter2.getVelocity() ;
      if (sh_vel < cur_vel) {
        cur_vel = sh_vel ; 
      }
    }
    
    
    while (sh_vel > 400) {
      sh_vel = shooter2.getVelocity() ;
    }
    
    
    telemetry.addData("Shooter2 velocity End", " " + cur_vel) ;
    telemetry.update() ;
    
    stopInclinedPlane();
    stopShooter();
    
    sleep(5000) ;
  }
  
  */
  private void testShooter1velocity() {
    
    runShooter(0.9);
    sleep(3000);
    
    runInclinedPlane(1.0);
    // assuming that the first ring is shot within 2 seconds
    // might need to use a sensor to check when the first one is being shot 
    shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    sleep(2000) ;
    double  sh_vel = 0.0 ; 
    sh_vel = shooter1.getVelocity() ;

    long startTime = 0; 
    double cur_vel = 0.0 ; 
    
    //startTime = rtime.milliseconds() ; 
    startTime = System.currentTimeMillis() ; 

    telemetry.addData("Shooter1 velocity Start", " " + sh_vel) ;
    
    cur_vel = sh_vel ; 

    while ((System.currentTimeMillis() - startTime) < 8000) {
      // find out how the velocity reduces 
      sh_vel = shooter1.getVelocity() ;
      if (sh_vel < cur_vel) {
        cur_vel = sh_vel ; 
      }
    }
    
    /*
    while (sh_vel > 400) {
      sh_vel = shooter2.getVelocity() ;
    }
    */
    
    telemetry.addData("Shooter1 velocity End", " " + cur_vel) ;
    telemetry.update() ;
    
    stopInclinedPlane();
    stopShooter();
    
    sleep(5000) ;
  }
  
  private void testNoodleIntake() {
    runInclinedPlane(1.0);
    sleep(1000) ;
    runNoodleIntake(1.0) ;
    sleep(1000) ;
    
    moveBackwardPower(1.0) ;
    sleep(5000) ;
   
    stopAllWheelsPower();
         
    stopNoodleIntake();
    stopInclinedPlane();
  }
  
  private void testIMU() {
    // it is assumed here the the IMU has been initialized
    
    // first reset and then get the angles 
    // reset here does not mean IMU reset. just storing it inside lastAngles and 
    // making global angles as zero 
    
    resetAngle();
    
    // display the values 
    getAngle() ;
    displayOrientationValues(lastAngles, "imu start") ;
    
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
    
    // Experimentally, we have established that the firstAngle value is the 
    // one that changes when the robot changes direction 
    // This is crucial.. 
    // Check REV robotics documentation.. it seems a bit off 
    
    sleep(20000) ;
  }
  
  private void testForwardIMU() {
    
    long startTime = System.currentTimeMillis() ; 
    
    resetAngle();

    while ((System.currentTimeMillis() - startTime) < 5000) {
      moveForwardIMU(1.0) ;
    }
    stopAllWheelsPower();
    
    sleep(5000) ;
  }
  
  private void testBackwardIMU() {
    
    long startTime = System.currentTimeMillis() ; 
    
    resetAngle();

    while ((System.currentTimeMillis() - startTime) < 5000) {
      moveForwardIMU(-1.0) ; // NOTE
    }
    stopAllWheelsPower();
    
    sleep(5000) ;
  }
  
  private void testStrafeRightIMU() {
    long startTime = System.currentTimeMillis() ; 
    
    resetAngle();

    while ((System.currentTimeMillis() - startTime) < 4000) {
      strafeRightIMU(1.0) ;
    }
    stopAllWheelsPower();
          
    sleep(5000) ;
    
  }
  
  private void testStrafeLeftIMU() {
    long startTime = System.currentTimeMillis() ; 
    
    resetAngle();

    while ((System.currentTimeMillis() - startTime) < 4000) {
      strafeRightIMU(-1.0) ;
      //strafeLeftPower(1.0);
    }
    stopAllWheelsPower();
    
    sleep(5000) ;
    
  }
  
  private void shootRingsFromStart(double plevel){

    // First get the shooter reved up 
    // Run it at the right power level so as to target the high goal
    runShooter(plevel);   
    // you can use the velocity to determine if it has revved up sufficiently 
    // but for now just sleep for some time  
    sleep(2000);  //warming up shooter
    
    
    // get the velocity of the shooter 
    double  cur_vel = 0.0;
    double shot_vel = 1900.0 ; // This is a fixed value, this can be a % of the initial velocity also
    
    // Start running the inclined plane 
    runInclinedPlane(1.0);
    
    // sometimes the first ring is not getting shot so move the lift gate 
    liftgate1.setPosition(1.0);

    // the first one will be shot given that the second one in line will push it 
    // determine when the first is shot 
    cur_vel = shooter1.getVelocity() ;
    while(cur_vel > shot_vel) {
      cur_vel = shooter1.getVelocity() ; 
    }
    // The first ring has been shot
    sleep(500) ; // just wait a bit for the action to complete
    
    liftgate2.setPosition(1.0);
    // For the second one, run the noodle so that it can push the third ring a bit 
    // and nudge the second ring to get going on the inclined plane
    runNoodleIntake(0.5); // note the power level   
    
    // we may want the lift gate2 to move the second one further along  
    
    // determine when the second one is shot using velocity as well 
    cur_vel = shooter1.getVelocity() ;
    while(cur_vel > shot_vel) {
      cur_vel = shooter1.getVelocity() ; 
    }
    // The second ring has been shot
    sleep(500) ; // just wait a bit for the action to complete

  
    // start a timer for the third one   
    long startTime = 0; 
    
    //startTime = rtime.milliseconds() ; 
    startTime = System.currentTimeMillis() ; 

    while ((System.currentTimeMillis() - startTime) < 8000) {
      // find out how the velocity reduces 
      // determine when the second one is shot using velocity as well 
      cur_vel = shooter1.getVelocity() ;
      if (cur_vel < shot_vel) {
        break ;
      }
    }
    // Either the timer ran out or the third ring got shot 
    sleep(400) ;
    
    // stop the rest of the system 
    liftgate2.setPosition(0.5); 
    liftgate1.setPosition(0.5);
    stopShooter();
    stopInclinedPlane();
    stopNoodleIntake();
    
  }

  private void strafeToPerimeterWallAndAdjust(double plevel){
    
    double wall_dist = 4.0 ; 
    
    // its a short distance so just strafe
    strafeLeftPower(plevel) ;
    
    double dist_lf, dist_lb ;
    
    dist_lf = distanceLeftFront.getDistance(DistanceUnit.INCH) ;
    dist_lb = distanceLeftBack.getDistance(DistanceUnit.INCH) ;
    
    while((dist_lf > wall_dist) && (dist_lb > wall_dist)) {
      // continue to strafe 
      dist_lf = distanceLeftFront.getDistance(DistanceUnit.INCH) ;
      dist_lb = distanceLeftBack.getDistance(DistanceUnit.INCH) ;
    }
    stopAllWheelsPower();
    
    // Adjust manually
    driveFR.setPower(1.0);
    driveBR.setPower(1.0);
    driveFL.setPower(0.0);
    driveBL.setPower(0.0);
    sleep(450);
    stopAllWheelsPower();
    
    /*
    // find which one registered the distance first and adjust accordingly
    dist_lf = distanceLeftFront.getDistance(DistanceUnit.INCH) ;
    dist_lb = distanceLeftBack.getDistance(DistanceUnit.INCH) ;
    if (dist_lf <= wall_dist) {
      if (dist_lb <= wall_dist) {
        // dont do anything for now 
        // think of adjusting later 
      } else {
        // need to adjust a bit so it is parallel to the perimeter wall  
        // adjust manually or move for a bit 
        driveFL.setPower(1.0);
        driveBL.setPower(1.0);
        driveFR.setPower(0.0);
        driveBR.setPower(0.0);
        
        dist_lb = distanceLeftBack.getDistance(DistanceUnit.INCH) ;
        while (dist_lb > wall_dist) {
          dist_lb = distanceLeftBack.getDistance(DistanceUnit.INCH) ; 
        }
        stopAllWheelsPower();
      }
    } else {
      if (dist_lb <= wall_dist) {
        // need to adjust a bit 
        driveFR.setPower(1.0);
        driveBR.setPower(1.0);
        driveFL.setPower(0.0);
        driveBL.setPower(0.0);
        
        dist_lf = distanceLeftFront.getDistance(DistanceUnit.INCH) ;
        while (dist_lf > wall_dist) {
          dist_lf = distanceLeftFront.getDistance(DistanceUnit.INCH) ; 
        }
        stopAllWheelsPower();
        
      } else {
        // that would be weird??
        // think of adjusting later 
      }
    }
    */
  }
  
  private void getSensorValues() {
    telemetry.addData("DistanceLeftFront", " " + distanceLeftFront.getDistance(DistanceUnit.INCH)) ;
    telemetry.addData("DistanceLeftBack", " " + distanceLeftBack.getDistance(DistanceUnit.INCH)) ;
    telemetry.addData("DistanceRight", " " + distanceRight.getDistance(DistanceUnit.INCH)) ;
    telemetry.addData("DistanceFront", " " + distanceFront.getDistance(DistanceUnit.INCH)) ;
    telemetry.addData("DistanceBack", " " + distanceBack.getDistance(DistanceUnit.INCH)) ;

    telemetry.addData("Color Front Left", " " +   launchLineLeft.alpha() ) ;
    telemetry.addData("Color Front Right", " " +   launchLineRight.alpha() ) ; 
    
    telemetry.update() ;
  }
  
  private void testSensors() {
    long startTime = 0; 
    
    //startTime = rtime.milliseconds() ; 
    startTime = System.currentTimeMillis() ; 

    while ((System.currentTimeMillis() - startTime) < 20000) {
      getSensorValues(); 
      sleep (200) ;
    }
  }
  
}