// Created by Vikram Kommera on 10/8/2021
// Mechanum Wheel Control
// Inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html



package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class robot16481 {
   /*********************************
    * Drive Declaration
    ****************/
   DcMotor motorFrontLeft;
   DcMotor motorBackLeft;
   DcMotor motorFrontRight;
   DcMotor motorBackRight;

   DistanceSensor leftRange;
   DistanceSensor rightRange;
   DistanceSensor backRange;

   //strafe constants
   final int STRAFE_RIGHT = 1;
   final int STRAFE_LEFT = 2;
   final int FORWARD = 3;
   final int REVERSE = 4;

   // This is for core hex motor with 60:1 gears
   final double TICKS_PER_REV = 28;
   //final double DRIVE_GEAR_REDUCTION = 54.5; // due to the 10% driver move gear from 60 to 54 (10% less)
   final double DRIVE_GEAR_REDUCTION = 60; // due to the 10% driver move gear from 60 to 54 (10% less)
   final double WHEEL_DIAMETER_MM = 101.6; // 25.4 * 4 inc
   final double COUNT_PER_MM = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_MM*3.1415);


   final int FIRST_SEG=1; // Move away from wall
   final int SECOND_SEG=2; // Move towards carousel
   final int THIRD_SEG=3; // Move toward shipping hub
   final int FORTH_SEG_1=4; // Strife toward shipping hub
   final int FORTH_SEG_2=5; // Strife toward shipping hub
   final int FORTH_SEG_3=6; // Strife toward shipping hub
   final int FIFTH_SEG=7;
   final int SIXTH_SEG=8;

   /*********************************
    * ARM  Declaration
    ****************/
   DcMotor armMotor;
   AnalogInput potMeter;
   Servo tbServo;

   // ARM top and bottom stop values for POT
   final double ARM_UPPER_STOP = 2.7;
   final double ARM_LOWER_STOP = 0.7;
   final double ARM_L1_UPPER_STOP= 1.18;
   final double ARM_L1_LOWER_STOP = 0.985;
   final double ARM_L2_UPPER_STOP= 1.56;
   final double ARM_L2_LOWER_STOP = 1.387;
   final double ARM_L3_UPPER_STOP= 1.948;
   final double ARM_L3_LOWER_STOP = 1.781;

   /*********************************
    * Claw Declaration
    ****************/
   Servo clawServo;
   DistanceSensor clawRange;

   // Servo close and open values
   final double CLAW_CLOSE =  0.58;
   final double CLAW_OPEN = 0.41;
   boolean CLAW_RANGE_ENABLE=true;

   /*********************************
    * Carouse Declaration
    ****************/
   DcMotor carouselMotor;
   DistanceSensor carouselRange;

   /*********************************
    * AI Declaration
    ****************/
   private static final String TFOD_MODEL_ASSET = "16481-TSE.tflite";
   private static final String[] LABELS = { "BLACK", "TSE"};

   private static final String VUFORIA_KEY =
           "ASh7iJr/////AAABmTWXFX0bzETGvTHXq6xLQpZR1ctRSGfgBfgdkTGPQ7S/Ie1VmeZpVEGmtSaG42XNyIMZwLcvRjIF4EeBjmLqixt+cKao65srQR/B3qb/qdYXsXgnFv1euNRS1+kb/u92uz8JuvDrUed4ivTVPZKL1rBLtc17LmOwAVPwM4ym+jTBYvkzAV06ahSMR9B5hc/WKBbpAOBfTa1i4hUPlEhiVnp3F3tkc6CTu2hiCRux9KY1mcx09eTHLxZoU2NYTwco1VPULtzaTA8rT8jSUwv6mdbZdaqd6zBzhsbJ33YRcdpZyw1clgZSXQU7JDetCoc/E25hOGP1hNNaqaKsozXlWkKJnsYb5FB09MiEj2HjeRbT";

   public VuforiaLocalizer vuforia;
   public WebcamName webcam;
   public TFObjectDetector tfod;


   HardwareMap hwMap   = null;
   Telemetry telemetry = null;

   public void hardwareSetup()
   {
      /*********************
       *   Drive Hardware Setup
       ************/
      motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
      motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
      motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
      motorBackRight = hwMap.dcMotor.get("motorBackRight");

      motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
      motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
      motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
      motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

      leftRange = hwMap.get(DistanceSensor.class, "leftRange");
      rightRange = hwMap.get(DistanceSensor.class, "rightRange");
      backRange = hwMap.get(DistanceSensor.class, "backRange");

      driverMode(DcMotor.RunMode.RUN_USING_ENCODER);

      /*********************
       *   ARM Hardware Setup
       ************/
      armMotor = hwMap.dcMotor.get("armMotor");
      tbServo = hwMap.servo.get("tbServo");
      potMeter = hwMap.analogInput.get("potMeter");

      armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      /*********************
       *   Carousel Hardware Setup
       ************/
      carouselMotor = hwMap.dcMotor.get("carouselMotor");
      carouselRange = hwMap.get(DistanceSensor.class, "carouselRange");

      /*********************
       *   Claw Hardware Setup
       ************/
      clawServo = hwMap.servo.get("clawServo");
      clawRange = hwMap.get(DistanceSensor.class, "clawRange");
   }

   /********************************************************************
    *   Drive Functions
    *********************************************/
   public void driverPower(double frontLeftPower, double frontRightPower,double backLeftPower, double backRightPower )
   {
      motorFrontLeft.setPower(frontLeftPower);
      motorFrontRight.setPower(frontRightPower);
      motorBackLeft.setPower(backLeftPower);
      motorBackRight.setPower(backRightPower);
   }

   public void driverMode(DcMotor.RunMode mode  )
   {
      motorFrontLeft.setMode(mode);
      motorFrontRight.setMode(mode);
      motorBackLeft.setMode(mode);
      motorBackRight.setMode(mode);
   }

   public void driveTarget(int frontLeftTarget, int frontRightTarget,int backLeftTarget, int backRightTarget)
   {
      motorFrontLeft.setTargetPosition(frontLeftTarget);
      motorFrontRight.setTargetPosition(frontRightTarget);
      motorBackLeft.setTargetPosition(backLeftTarget);
      motorBackRight.setTargetPosition(backRightTarget);
   }

   public void drive(double x, double y, double rx)
   {
      double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
      double frontLeftPower = (y + x + rx) / denominator;
      double backLeftPower = (y - x + rx) / denominator;
      double frontRightPower = (y - x - rx) / denominator;
      double backRightPower = (y + x - rx) / denominator;

      driverPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
      telemetry.addData("Power setting:", "\n " +
                      "Front left: %f\n, " +
                      "Front right: %f\n, " +
                      "Back left: %f\n, " +
                      "Back right %f \n",
              frontLeftPower, frontRightPower,
              backLeftPower, backRightPower);
   }

   double current_value=1.0;
   public double getSensorRange(DistanceSensor sensor, String name)
   {
      double sensor_val = sensor.getDistance(DistanceUnit.CM);
      if (sensor_val<200) {
         current_value = sensor_val;
      }

      telemetry.addData(name,", Range Sensor Value: %f, %f", current_value, sensor_val);
      telemetry.addData("# Left motor position:",motorFrontLeft.getCurrentPosition());
      telemetry.update();
      return current_value;
   }

   public void sensorDrive(int segment) throws InterruptedException
   {
      // driverMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      telemetry.addData("Segment", segment);
      telemetry.update();
      switch (segment)
      {
         case FIRST_SEG: // Move way from wall
            drive(0.8, 0, 0);
            while ( getSensorRange(leftRange, "leftRange") <10 )
            {
               Thread.sleep(30);
            }
            break;
         case SECOND_SEG: // Move towards carousel
            driverPower(-0.5, -0.5, -0.5, -0.5);
            while( getSensorRange(backRange, "backRange")  >10 &&
                    getSensorRange(carouselRange, "carouselRange") >6 )
            {
               Thread.sleep(50);
            }
            break;
         case THIRD_SEG: // Move towards shipping hub (backRange.getDistance(DistanceUnit.CM) > 8 &&
            driverMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driverMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driverPower(0.5, 0.5, 0.5, 0.5);
            // (getSensorRange(backRange, "backRange") < 125)
            while ( motorFrontRight.getCurrentPosition()<6000) {
               Thread.sleep(10);
            }
            while ( getSensorRange(rightRange, "rightRange") > 100 ) {
               Thread.sleep(10);
            }
         case FORTH_SEG_1:
            drive(0.8, 0, 0);
            while ( getSensorRange(leftRange, "leftRange") < 53 ) {
               Thread.sleep(30);
            }
            break;
         case FORTH_SEG_2:
            drive(0.8, 0, 0);
            while ( getSensorRange(leftRange, "leftRange") < 54 ) {
               Thread.sleep(30);
            }
            break;
         case FORTH_SEG_3:
            drive(0.8, 0, 0);
            while ( getSensorRange(leftRange, "leftRange") < 55 ) {
               Thread.sleep(30);
            }
            break;
         case FIFTH_SEG:
            drive(-0.8, 0, 0);
            while ( getSensorRange(leftRange, "leftRange") >8 ) {
               Thread.sleep(30);
            }
            break;
         case SIXTH_SEG:
            driverMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driverMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driverPower(0.5, 0.5, 0.5, 0.5);
            while ( motorFrontRight.getCurrentPosition()<6000) {
               Thread.sleep(10);
            }
            break;
      }

      driverPower(0, 0, 0, 0);
   }

   public void encoderDrive(double leftInches, double rightInches, double speed, int direction ) throws InterruptedException {

      double leftMM = leftInches * 25.4;
      double rightMM = rightInches * 25.4;

      driverMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      int frontLeftTarget = motorFrontLeft.getCurrentPosition() + (int)(leftMM*COUNT_PER_MM);
      int backLeftTarget = motorBackLeft.getCurrentPosition() + (int)(leftMM*COUNT_PER_MM);
      int frontRightTarget = motorFrontRight.getCurrentPosition() + (int)(rightMM*COUNT_PER_MM);
      int backRightTarget = motorBackRight.getCurrentPosition() + (int)(rightMM*COUNT_PER_MM);

      switch (direction)
      {
         case FORWARD:
            driveTarget(frontLeftTarget,frontRightTarget,backLeftTarget,backRightTarget);
            break;
         case REVERSE:
            driveTarget(frontLeftTarget*-1,frontRightTarget*-1,
                    backLeftTarget*-1,backRightTarget*-1);
            break;
         case STRAFE_LEFT:
            driveTarget(frontLeftTarget * -1,frontRightTarget,
                    backLeftTarget,backRightTarget*-1);
            break;
         case STRAFE_RIGHT:
            driveTarget(frontLeftTarget,frontRightTarget*-1,
                    backLeftTarget*-1,backRightTarget);
            break;
      }
      driverMode(DcMotor.RunMode.RUN_TO_POSITION);
      driverPower(speed, speed, speed, speed);

      while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||
              motorFrontRight.isBusy() || motorBackRight.isBusy())
      {
         Thread.sleep(50);
      }

      driverPower(0, 0, 0, 0);
      driverMode(DcMotor.RunMode.RUN_USING_ENCODER);
   }

   /********************************************************************
    *   Carousel Functions
    *********************************************/
   public void carousel(double speed, int direction) {
         carouselMotor.setPower(speed * direction);
   }

   public void carouselAuto(int direction) throws InterruptedException {
      carousel(0.75, direction);
      Thread.sleep(4000);
      carousel(0,direction);
   }

   /****************************************************************
    ARM Code
    ***********************************/
   public void moveARM(double armPower)
   {
      if ((potMeter.getVoltage() < ARM_LOWER_STOP && armPower > 0) ||
              (potMeter.getVoltage() > ARM_UPPER_STOP && armPower < 0)) {
         armMotor.setPower(0);
      } else {
         armMotor.setPower(armPower);
      }
   }

   public  void setArm(int level)
   {
      double upperValue=ARM_L1_UPPER_STOP;
      double lowerValue=ARM_L1_LOWER_STOP;
      final double ARM_SPEED = 0.65;

      if(level == 1) { upperValue = ARM_L1_UPPER_STOP; lowerValue = ARM_L1_LOWER_STOP; }
      else if(level == 2) { upperValue = ARM_L2_UPPER_STOP; lowerValue = ARM_L2_LOWER_STOP; }
      else if(level == 3) { upperValue = ARM_L3_UPPER_STOP; lowerValue = ARM_L3_LOWER_STOP; }
      else {return;}

      while (potMeter.getVoltage() > upperValue || potMeter.getVoltage() < lowerValue)
      {
         if (potMeter.getVoltage() > upperValue)  { moveARM(ARM_SPEED); }
         else if (potMeter.getVoltage() < lowerValue) { moveARM(ARM_SPEED * -1); }
      }
      armMotor.setPower(0);
   }

   /*********************************
    * Claw Functions
    ****************/
   public void setClawPosition(double position)
   {
      clawServo.setPosition(position);
   }

   /*********************************
    * AI Functions
    ****************/
   /* Initialize the TensorFlow Object Detection engine.  */
   public void initTfod() {
      /* Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
       */
      VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
      parameters.vuforiaLicenseKey = VUFORIA_KEY;

      // Indicate that we wish to be able to switch cameras.
      webcam = hwMap.get(WebcamName.class, "RacerCam");
      parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam);

      //  Instantiate the Vuforia engine
      vuforia = ClassFactory.getInstance().createVuforia(parameters);
      int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
              "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
      TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
      tfodParameters.minResultConfidence = 0.6f;
      tfodParameters.isModelTensorFlow2 = true;
      tfodParameters.inputSize = 320;

      tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
      tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

      tfod.activate();
      tfod.setZoom(1.5, 16.0/9.0);
   }

   public int elementPosition( ) {
      int position = 3;

      if (tfod != null) {
         int i = 0;
         List<Recognition> recognitions = tfod.getRecognitions();
         for (i=0; i>10000;i++)
         {
            if(recognitions.size()>0)
            {
               telemetry.addData("size in for", recognitions.size());
               telemetry.update();
               break;
            }
            recognitions = tfod.getRecognitions();
         }
         telemetry.addData("i=", i);
         telemetry.update();
         for (Recognition recognition : recognitions) {
               telemetry.addData("left", recognition.getLeft());
               telemetry.update();
               if( recognition.getLabel() == "TSE")
                  if (0.0 < recognition.getLeft() && recognition.getLeft() < 200)
                  {
                     position = 3;
                  }
               else if (200 < recognition.getLeft() && recognition.getLeft() < 400)
                  {
                     position = 2;
                  }
               else
                  {
                     position = 1;
                  }
            }
         telemetry.addData("object size: ",recognitions.size());
         telemetry.addData("position", position);
         telemetry.update();
      }
      else
      {
         telemetry.addData("tfod is null", position);
         telemetry.update();
      }
      return position;
   }
}

