package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="COMP_B_AUTO_LT_OB", group="Robot")


public class COMP_B_AUTO_LT_OB extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotorEx bl = null, fl = null, fr = null, br = null;
    public DcMotorEx armMotor = null; // The arm motor
    public DcMotorEx liftMotor = null; // The lift motor
    public CRServo intake = null; // The active intake servo
    public Servo wrist = null; // The wrist servo


    /* This constant is the number of encoder ticks for each degree of rotation of the arm.*/
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to. */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_HIGH  = 95 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;
    final double ARM_DOWNBIT               = 65 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLIP               = 51 * ARM_TICKS_PER_DEGREE;
    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.2; //Wrist folded so that it is tucked into the robot to confirm to 18in sizing rule
    final double WRIST_FOLDED_OUT  = 0.52;


    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    

    // Calculates lift ticks per mm using motor encoder
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    /* Variables that are used to set the arm to a specific position */
    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 455 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_COLLAPSED;

    SparkFunOTOS myOtos;
    
    @Override
    public void runOpMode() {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        /* Define and Initialize Motors */
        bl = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        fl = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        fr = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        br = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        liftMotor       = hardwareMap.get(DcMotorEx.class, "slideMotor"); //the lift motor aka slide
        armMotor        = hardwareMap.get(DcMotorEx.class, "armMotor"); //the arm motor

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setCurrentAlert(5,CurrentUnit.AMPS);
        liftMotor.setCurrentAlert(5,CurrentUnit.AMPS);

         /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
         Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
         If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        //Initialize timer object
        

        //Configure OTOS
        configureOtos();

        /* Wait for the game driver to press play */
        waitForStart();

        //Initialize OTOS
        
        // Reset the timer
        

        if (opModeIsActive()){
        ElapsedTime timer2 = new ElapsedTime();
            

            //Hold wrist in while raising to barrier position
            wrist.setPosition(WRIST_FOLDED_IN);

            //Raise arm to barrier position
            liftPosition = 0;
            liftMotor.setTargetPosition ((int)liftPosition);
            liftMotor.setVelocity(100);
            
            armPosition = ARM_CLEAR_BARRIER;
            armMotor.setTargetPosition ((int)armPosition);
            armMotor.setVelocity(600);
            while (armMotor.isBusy() && opModeIsActive()) {
                telemetry.addData("Arm Variable", armPosition);
                telemetry.addData("Arm Target Position: ", armMotor.getTargetPosition());
                telemetry.addData("Arm Current Position: ", armMotor.getCurrentPosition());
                telemetry.addData("ArmMotor Current:",armMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
            }
            

            //Fold wrist out
            wrist.setPosition(WRIST_FOLDED_IN);
            sleep(1000); //pause before moving

            //Strafe right
           movement("forward",0.2,20,10);
           movement("stRight", 0.2, 5, 10);
           sleep(500);
           armPosition = ARM_SCORE_SAMPLE_IN_HIGH;
           armMotor.setTargetPosition ((int)armPosition);
            armMotor.setVelocity(600);
           while (armMotor.isBusy() && opModeIsActive()) {
                telemetry.addData("Arm Variable", armPosition);
                telemetry.addData("Arm Target Position: ", armMotor.getTargetPosition());
                telemetry.addData("Arm Current Position: ", armMotor.getCurrentPosition());
                telemetry.addData("ArmMotor Current:",armMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
            }
            movement("forward",0.2,9.5,10);
            
            armPosition = ARM_CLIP;
            armMotor.setTargetPosition ((int)armPosition);
            armMotor.setVelocity(1500);
            while (armMotor.isBusy() && opModeIsActive()) {
                intake.setPower(INTAKE_COLLECT);
                telemetry.addData("Arm Variable", armPosition);
                telemetry.addData("Arm Target Position: ", armMotor.getTargetPosition());
                telemetry.addData("Arm Current Position: ", armMotor.getCurrentPosition());
                telemetry.addData("ArmMotor Current:",armMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
            }
            intake.setPower(INTAKE_OFF);
            sleep(1000);
            
            armPosition = ARM_DOWNBIT;
            armMotor.setTargetPosition ((int)armPosition);
            armMotor.setVelocity(1500);
            while (armMotor.isBusy() && opModeIsActive()) {
                intake.setPower(INTAKE_COLLECT);
                telemetry.addData("Arm Variable", armPosition);
                telemetry.addData("Arm Target Position: ", armMotor.getTargetPosition());
                telemetry.addData("Arm Current Position: ", armMotor.getCurrentPosition());
                telemetry.addData("ArmMotor Current:",armMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
            }
            intake.setPower(INTAKE_OFF);
            sleep(500);
            intake.setPower(INTAKE_DEPOSIT);
            sleep(500);
            intake.setPower(INTAKE_COLLECT);
            sleep(500);
            intake.setPower(INTAKE_DEPOSIT);
            sleep(500);
            intake.setPower(INTAKE_OFF);
            sleep(500);
            armPosition = ARM_DOWNBIT;
            armMotor.setTargetPosition ((int)armPosition);
            armMotor.setVelocity(1500);
            
            while (armMotor.isBusy() && opModeIsActive()) {
                intake.setPower(INTAKE_DEPOSIT);
                telemetry.addData("Arm Variable", armPosition);
                telemetry.addData("Arm Target Position: ", armMotor.getTargetPosition());
                telemetry.addData("Arm Current Position: ", armMotor.getCurrentPosition());
                telemetry.addData("ArmMotor Current:",armMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
            }
            movement("backward",0.2,-28,10);
            movement("stRight", 0.5, 37,10);
           //movement("stRight",0.2,15,10);
           armPosition = ARM_CLEAR_BARRIER;
            armMotor.setTargetPosition ((int)armPosition);
            armMotor.setVelocity(1500);
            while (armMotor.isBusy() && opModeIsActive()) {
                intake.setPower(INTAKE_COLLECT);
                telemetry.addData("Arm Variable", armPosition);
                telemetry.addData("Arm Target Position: ", armMotor.getTargetPosition());
                telemetry.addData("Arm Current Position: ", armMotor.getCurrentPosition());
                telemetry.addData("ArmMotor Current:",armMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
            }
            intake.setPower(INTAKE_OFF);
            sleep(1000);
            
        }
    }
    private void stopMotion(){
        fr.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot.
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 2, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }



//attempting to combine all movements into one big method, move = what to do, speed = how fast, d = distance, time = time before forced stop.
public void movement(String move, double speed, double  d, int time){
   //checking for the actual direction and movement
   configureOtos();
   ElapsedTime timer = new ElapsedTime();
   SparkFunOTOS.Pose2D pos;
        pos = myOtos.getPosition();
   if(move == "forward"){
      timer.reset();
      while(pos.y<d || timer.seconds()>time){
         fr.setPower(speed);
         fl.setPower(speed);
         bl.setPower(speed);
         br.setPower(speed); 
         pos =myOtos.getPosition();
         telemetry.addData("X coordinate", pos.x);
         telemetry.addData("Y coordinate", pos.y);
         telemetry.addData("Heading angle", pos.h);
         // Update the telemetry on the driver station
         telemetry.update();
      }
      //stopping motor at a certain point
         stopMotion(); 
   }

   if(move == "backward"){
      timer.reset();
      while(pos.y>d || timer.seconds()>time){
         fr.setPower(-1* speed);
         fl.setPower(-1*speed);
         bl.setPower(-1*speed);
         br.setPower(-1*speed);
      pos =myOtos.getPosition();
         telemetry.addData("X coordinate", pos.x);
         telemetry.addData("Y coordinate", pos.y);
         telemetry.addData("Heading angle", pos.h);
         // Update the telemetry on the driver station
         telemetry.update();
      }
      //stopping motor at a certain point
         stopMotion(); 
   }

   if(move == "stLeft"){
      timer.reset();
      while(pos.x>d || timer.seconds()>time){
         fr.setPower(speed);
         fl.setPower(-1*speed);
         bl.setPower(speed);
         br.setPower(-1*speed);
      pos =myOtos.getPosition();
         telemetry.addData("X coordinate", pos.x);
         telemetry.addData("Y coordinate", pos.y);
         telemetry.addData("Heading angle", pos.h);
         // Update the telemetry on the driver station
         telemetry.update();
      }
      //stopping motor at a certain point
         stopMotion(); 
   }

   if(move == "stRight"){
      timer.reset();
      while(pos.x<d || timer.seconds()>time){
          fr.setPower(-1*speed);
          fl.setPower(speed);
          bl.setPower(-1*speed);
          br.setPower(speed);
     pos =myOtos.getPosition();
         telemetry.addData("X coordinate", pos.x);
         telemetry.addData("Y coordinate", pos.y);
         telemetry.addData("Heading angle", pos.h);
         // Update the telemetry on the driver station
         telemetry.update();
      }
      //stopping motor at a certain point
         stopMotion(); 
   }

   if(move == "rRight"){
      timer.reset();
      while(pos.h<d || timer.seconds()>time){
          fr.setPower(-1*speed);
          fl.setPower(speed);
          bl.setPower(speed);
          br.setPower(-1*speed);
      pos =myOtos.getPosition();
         telemetry.addData("X coordinate", pos.x);
         telemetry.addData("Y coordinate", pos.y);
         telemetry.addData("Heading angle", pos.h);
         // Update the telemetry on the driver station
         telemetry.update();
      }
      //stopping motor at a certain point
         stopMotion(); 
   }

   if(move == "rLeft"){
      timer.reset();
      while(pos.h>d || timer.seconds()>time){
          fr.setPower(speed);
          fl.setPower(-1*speed);
          bl.setPower(-1*speed);
          br.setPower(speed);
      pos =myOtos.getPosition();
         telemetry.addData("X coordinate", pos.x);
         telemetry.addData("Y coordinate", pos.y);
         telemetry.addData("Heading angle", pos.h);
         // Update the telemetry on the driver station
         telemetry.update();
      }
      //stopping motor at a certain point
         stopMotion(); 
   }
   
}
}


