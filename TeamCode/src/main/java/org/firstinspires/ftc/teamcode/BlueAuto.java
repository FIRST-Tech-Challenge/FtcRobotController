package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.logging.Level;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.DcMotorEx; 
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

//import com.qualcomm.robotcore.hardware;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware9010;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels;

@Autonomous(name = "BlueAuto")
public class BlueAuto extends LinearOpMode {

  Hardware9010 hdw;
  MecanumWheels robot;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    hdw = new Hardware9010(hardwareMap); //init hardware
    hdw.createHardware();
    robot = new MecanumWheels();
        
    hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    hdw.Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    hdw.Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    hdw.Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    hdw.wheelFrontRight.setDirection(DcMotor.Direction.FORWARD);
    hdw.wheelBackRight.setDirection(DcMotor.Direction.FORWARD);
    hdw.wheelBackLeft.setDirection(DcMotor.Direction.REVERSE);
    hdw.wheelFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        
        
    hdw.Vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    boolean shooterEnable = false;
    boolean TurretLeft = false;
    boolean TurretRight = false;
    boolean TurretUp = false;
    boolean TurretDown = false;
    boolean SpinnerLeft = false;
    boolean SpinnerRight = false;
    boolean EncoderUp = false;
    boolean EncoderDown = false;
    boolean fakeauto = true;
    boolean Turret = false;
    boolean Vertical = true;
    boolean autolevel = false;
    boolean autorotate = false;
    boolean intakeopen = false;
    boolean ManTurretRight = false;
    boolean ManTurretLeft = false;
    boolean ManTurretUp = false;
    boolean ManTurretDown = false;
    boolean TargetingOn = false;
    boolean TargetingOff = false;
    boolean TargetingRight = false;
    boolean TargetingLeft = false;
    boolean LevelUp = false;
    boolean LevelMid = false;
    boolean LevelDown = false;
    boolean LevelOff = false;
    boolean IntakeIn = false;
    boolean IntakeOut = false;
    boolean highmode = true;
    boolean sharemode = false;
    boolean slowDriveMode = false;
    boolean slowShootMode = false;
    boolean CarouselSpinner = false;
    int steps = 0;
    int currentSlide = 0;
    double powerDrivePercentage = 1.0;
    double powerShootPercentage = 100;
    double offsetwheels = 0.0;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    
    // Put initialization blocks here.
    hdw.grabberclaw.setPosition(1.0);
    hdw.Encoders.setPosition(0.35);
    
    waitForStart();
    AllTurret(500, 1.0, 2000, 1.0, 1100, 0.65);
    hdw.grabberclaw.setPosition(0.7);
    sleep(200);
    Slide(0, 1.0, 0, 1.0, 1100, 0.65);
    DriveIn(0.6);
  }

  /**
   * Describe this function...
   */
  private void AllTurret(int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed, 
            int VerticalTarget, double VerticalSpeed) 
  {
    hdw.Turret.setTargetPosition(TurretTarget);
    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hdw.Turret.setPower(TurretSpeed);
    hdw.Slide.setTargetPosition(SlideTarget);
    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hdw.Slide.setPower(SlideSpeed);
    hdw.Vertical.setTargetPosition(VerticalTarget);
    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hdw.Vertical.setPower(VerticalSpeed);
    while (opModeIsActive() && (hdw.Turret.isBusy() || hdw.Slide.isBusy() || hdw.Vertical.isBusy())) 
    {
      idle();
      telemetry.addData("lfWheel", hdw.wheelFrontLeft.getCurrentPosition());
      telemetry.addData("rfWheel", hdw.wheelFrontRight.getCurrentPosition());
      //telemetry.addData("strafeWheel", hdw.wheelStrafe.getCurrentPosition());
    }
  }
  private void DriveIn(double driveSpeed) 
  {
      double offsetwheels = (driveSpeed * 0.03333333333);
    if (hdw.wheelFrontRight.getCurrentPosition() > hdw.wheelFrontLeft.getCurrentPosition())
    {
        hdw.wheelFrontRight.setPower(driveSpeed);
        hdw.wheelBackRight.setPower(driveSpeed);
        hdw.wheelFrontLeft.setPower(driveSpeed - offsetwheels);
        hdw.wheelBackLeft.setPower(driveSpeed - offsetwheels);
    }
    else if (hdw.wheelFrontRight.getCurrentPosition() < hdw.wheelFrontLeft.getCurrentPosition())
    {
        hdw.wheelFrontRight.setPower(driveSpeed - offsetwheels);
        hdw.wheelBackRight.setPower(driveSpeed - offsetwheels);
        hdw.wheelFrontLeft.setPower(driveSpeed);
        hdw.wheelBackLeft.setPower(driveSpeed);
    }
    else 
    {   
        hdw.wheelFrontRight.setPower(driveSpeed);
        hdw.wheelBackRight.setPower(driveSpeed);
        hdw.wheelFrontLeft.setPower(driveSpeed);
        hdw.wheelBackLeft.setPower(driveSpeed);
    }
    while (opModeIsActive() && (hdw.wheelFrontRight.getCurrentPosition() < -200 || hdw.wheelFrontLeft.getCurrentPosition() < -200))
    {
      idle();
      telemetry.addData("lfWheel", hdw.wheelFrontLeft.getCurrentPosition());
      telemetry.addData("rfWheel", hdw.wheelFrontRight.getCurrentPosition());
      //telemetry.addData("strafeWheel", hdw.wheelStrafe.getCurrentPosition());
    }
  }
  private void Slide(int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed, 
            int VerticalTarget, double VerticalSpeed) 
  {
    hdw.Turret.setTargetPosition(TurretTarget);
    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hdw.Turret.setPower(TurretSpeed);
    hdw.Slide.setTargetPosition(SlideTarget);
    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hdw.Slide.setPower(SlideSpeed);
    hdw.Vertical.setTargetPosition(VerticalTarget);
    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hdw.Vertical.setPower(VerticalSpeed);
    while (opModeIsActive() && (hdw.Slide.getCurrentPosition() > 1000))
    {
      idle();
      telemetry.addData("lfWheel", hdw.wheelFrontLeft.getCurrentPosition());
      telemetry.addData("rfWheel", hdw.wheelFrontRight.getCurrentPosition());
      //telemetry.addData("strafeWheel", hdw.wheelStrafe.getCurrentPosition());
    }
  }
  private void AllTurretHold(int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed, 
            int VerticalTarget, double VerticalSpeed) 
  {
    hdw.Turret.setTargetPosition(TurretTarget);
    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hdw.Turret.setPower(TurretSpeed);
    hdw.Slide.setTargetPosition(SlideTarget);
    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hdw.Slide.setPower(SlideSpeed);
    hdw.Vertical.setTargetPosition(VerticalTarget);
    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hdw.Vertical.setPower(VerticalSpeed);
    while (opModeIsActive()) 
    {
      idle();
      telemetry.addData("lfWheel", hdw.wheelFrontLeft.getCurrentPosition());
      telemetry.addData("rfWheel", hdw.wheelFrontRight.getCurrentPosition());
      //telemetry.addData("strafeWheel", hdw.wheelStrafe.getCurrentPosition());
    }
  }
    
}