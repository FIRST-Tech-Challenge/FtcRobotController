/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous

public class StartNew extends LinearOpMode {
    private DcMotor BL;
    private DcMotor BR;
    private Blinker control_Hub;
    private DcMotor Elbow;
    private Blinker expansion_Hub_2;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor LArm;
    private Servo LClaw;
    private Servo Plane;
    private DcMotor RArm;
    private Servo RClaw;
    private HardwareDevice Webcam_1;
    private Servo Wrist;
    private IMU imu;
    private ColorSensor fSensor;
    private DistanceSensor distSensor;
    private double Green;
    private double Blue;
    private double Red;
    NormalizedColorSensor colorSensor;

    private static final boolean USE_WEBCAM = true; 
    private final String[] LABELS = {"Blue", "Red"};
    
    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private VisionPortal myVisionPortal;

    @Override
    public void runOpMode() {
        initDoubleVision();
        Wrist = hardwareMap.get(Servo.class, "Wrist");
    FR = hardwareMap.get(DcMotor.class, "FR");
    BR = hardwareMap.get(DcMotor.class, "BR");
    FL = hardwareMap.get(DcMotor.class, "FL");
    BL = hardwareMap.get(DcMotor.class, "BL");
    Elbow = hardwareMap.get(DcMotor.class, "Elbow");
    RArm = hardwareMap.get(DcMotor.class, "RArm");
    LArm = hardwareMap.get(DcMotor.class, "LArm");
    RClaw = hardwareMap.get(Servo.class, "RClaw");
    LClaw = hardwareMap.get(Servo.class, "LClaw");
    Plane = hardwareMap.get(Servo.class, "Plane");
    fSensor = hardwareMap.get(ColorSensor.class, "fSensor");
    imu = hardwareMap.get(IMU.class, "imu");
    distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");

    waitForStart();
        // This OpMode loops continuously, allowing the user to switch between
        // AprilTag and TensorFlow Object Detection (TFOD) image processors.
        while (!isStopRequested())  {

            

           
                
            //myVisionPortal.setProcessorEnabled(aprilTag, false);
            //myVisionPortal.setProcessorEnabled(tfod, false);

            sleep(20);

        }   // end while loop

    }   // end method runOpMode()


    /**
     * Initialize AprilTag and TFOD.
     */
    private void initDoubleVision() {
        
        aprilTag = new AprilTagProcessor.Builder()
            .build();

    
        TfodProcessor.Builder tensorBuilder;
        VisionPortal.Builder tensorVisionPortal;

    // First, create a TfodProcessor.Builder.
    tensorBuilder = new TfodProcessor.Builder();
    // Set the name of the file where the model can be found.
    tensorBuilder.setModelFileName("squish.tflite");
    // Set the full ordered list of labels the model is trained to recognize.
    tensorBuilder.setModelLabels(LABELS);
    // Set the aspect ratio for the images used when the model was created.
    //tensorBuilder.setModelAspectRatio(16 / 9);
    // Create a TfodProcessor by calling build.
    tfod = tensorBuilder.build();
    // Next, create a VisionPortal.Builder and set attributes related to the camera.

        
    myVisionPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .addProcessors(tfod, aprilTag)
        .build();
        
    }   // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()



    private void scanTfod(){
      List<Recognition> currentRecognitions = tfod.getRecognitions();

      for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
      }
    }
    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    private void tfodFind();{
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            double conf = (recognition.getConfidence() * 100);
            
        }
    }
    //puts the arm down
  public void armDown(){
    Wrist.setPosition(1);
    sleep(50);
    Elbow.setTargetPosition(0);
    Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Elbow.setPower(0.6);
    LArm.setTargetPosition(-5);
    RArm.setTargetPosition(-5);
    LArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LArm.setPower(0.25);
    RArm.setPower(0.25);
  }//end armDown
       
  //moves the arms to scoring position 
  public void score(){
    Elbow.setTargetPosition(-440);
    Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Elbow.setPower(0.5);
    LArm.setTargetPosition(-125);
    RArm.setTargetPosition(-125);
    LArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LArm.setPower(0.3);
    RArm.setPower(0.3);
    Wrist.setPosition(0.35);
  }//end score
        
  //method for keeping the code cleaner and for grouping the initialization commands        
  public void initialize() {
    FR.setDirection(DcMotor.Direction.FORWARD);
    BR.setDirection(DcMotor.Direction.FORWARD);
    FL.setDirection(DcMotor.Direction.FORWARD);
    BL.setDirection(DcMotor.Direction.FORWARD);
    FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LArm.setDirection(DcMotor.Direction.REVERSE);
    RArm.setDirection(DcMotor.Direction.FORWARD);
    LArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Elbow.setDirection(DcMotor.Direction.FORWARD);
    LClaw.setDirection(Servo.Direction.REVERSE);
    LClaw.scaleRange(0.45, 0.55);
    RClaw.scaleRange(0.01, 0.12);
    Wrist.scaleRange(0.19, 0.75);
    resetEncoders();
    imu.resetYaw();
  }//end initialize
        
  //method for setting the colors to a variable
  public void scan(){
    Red = fSensor.red();
    Blue = fSensor.blue();
    Green = fSensor.green();
  }//end scan
       
  //method to stop the motors 
  public void noMove(){
    FL.setPower(0);
    FR.setPower(0);
    BL.setPower(0);
    BR.setPower(0);
  }//end noMove
        
        
  /*
  -------------------------------------------------
               Helper Methods (Start)
  -------------------------------------------------
  These methods are used many times, they help reduce redundancy
  */
    
  //Sets the mode of the motors to RUN_TO_POSITION
  //Typically used with moving for a specific duration
  public void runToPos(){
    FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }//end runToPos
    
  //Stops and resets the encoder value stored in the motors
  //Typically used to refresh the encoder values to make it easier to code
  public void resetEncoders(){
    FL.setPower(0);
    FR.setPower(0);
    BL.setPower(0);
    BR.setPower(0);
    FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }//end resetEncoders
    
    
    
  //Makes the motors turn, so the robot turns
  //Positive is left, negative is right
  public void turning(double speed){
    BL.setPower(speed);
    BR.setPower(speed);
    FL.setPower(speed);
    FR.setPower(speed);
  }//end turning
    
  //Makes the motors turn, so the robot strafes
  //Positive is left, negative is right
  public void strafe(double speed){
    FL.setPower(speed);
    FR.setPower(-speed);
    BL.setPower(-speed);
    BR.setPower(speed);
  }//end strafe
    
  //Makes the motors turn, so the robot moves
  //Positive is forward, negative is left
  public void move(double speed){
    FL.setPower(-speed);
    FR.setPower(speed);
    BL.setPower(-speed);
    BR.setPower(speed);
  }//end move
    
  //Sets the target position of the motors in preparation to move
  public void moveDuration(int duration){
    FL.setTargetPosition(-duration);
    FR.setTargetPosition(duration);
    BL.setTargetPosition(-duration);
    BR.setTargetPosition(duration);
  }//end moveDuration
    
  //Sets the target position of the motors in preparation to strafe
  public void strafeDuration(int duration){
    FL.setTargetPosition(duration);
    FR.setTargetPosition(duration);
    BL.setTargetPosition(-duration);
    BR.setTargetPosition(-duration);
  }//end strafeDuration
    
  //Sets the target position of the motors in preparation to turn
  public void turnDuration(int duration){
    FL.setTargetPosition(duration);
    FR.setTargetPosition(duration);
    BL.setTargetPosition(duration);
    BR.setTargetPosition(duration);
  }
    
  //Uses the turning() to set a power to the motors, so the robot moves
  //It's set up this way to reduce confusion of what is happening
  public void motorSpeed(double speed){
    turning(speed);
  }//end motorSpeed
    
  //Makes the robot wait until it is done moving
  //Then, it resets the encoder values in preparation for the next movement
  private void delayReset(){
    //Wait while the BL motor is still turning and the OP mode is still active
    while(BL.isBusy() && opModeIsActive()){
      telemetry.update();
    }//end while
    resetEncoders();
  }//end delayReset
    
  //Returns the Yaw angle of the IMU
  //Yaw is the rotation about the Y axis
  //Used to reduce redundant code
  private Double getAngle(){
    return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
  }//end getAngle
    
    
  //Moves the robot forward for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void forward(int duration,double speed,Boolean delay){
    moveDuration(duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end forward
    
  //The robot strafes left for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void strafeLeft(int duration,double speed,Boolean delay){
    strafeDuration(duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end strafeLeft
    
  //The robot strafes right for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void strafeRight(int duration,double speed,Boolean delay){
    strafeDuration(-duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end strafeRight
    
  //Moves the robot turns left for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void turnLeft(int duration,double speed,Boolean delay){
    turnDuration(duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end turnLeft
    
  //Moves the robot turns right for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void turnRight(int duration,double speed,boolean delay){
    turnDuration(-duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end turnRight
    
  //Moves the robot backwards for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void backwards(int duration,double speed,boolean delay){
    moveDuration(-duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end backwards
    
  //Makes the robot turn to a specified angle
  //The angle depends on the last time the IMU values were reset
  //The robot turns until the robot's angle is 0.5 off the specified angle
  public void IMUTurn(double Angle,double turnSpeed){
    double Min = Angle - 0.5;
    double Max = Angle + 0.5;
    while(!(getAngle() > Min && getAngle() < Max) && opModeIsActive()){
      if(getAngle() < Min){
                
        //turns left
        turning(turnSpeed);
                
        while(!(getAngle() > Min && getAngle() < Max) && opModeIsActive()){
          telemetry.update();
          if(getAngle() > Max){
            break;
          }//end if
        }//end while
        turning(0);
      }//end if
      
      else if(getAngle() > Max){
        //turns right
        turning(-turnSpeed);
        while(!(getAngle() > Min && getAngle() < Max) && opModeIsActive()){
          telemetry.update();
          if(getAngle() < Min){
            break;
          }//end if
        }//end while
        turning(0);
      }//end elseif
    }//end while
    
    resetEncoders();
    }//end IMUTurn
}   // end class
