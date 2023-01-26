package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ColorSensorRangeTest extends LinearOpMode {

  private DistanceSensor sensorRange;
  ColorSensor color;
  /*
  Servo bucket;
  DcMotor right_drive;
  DcMotor left_drive;
  DcMotor back_right_drive;
  DcMotor back_left_drive;
  DcMotor Intake;
  DcMotor Slide;
  DcMotor Caro;
  */

  @Override
  public void runOpMode() throws InterruptedException {
    color = hardwareMap.get(ColorSensor.class, "color");
    /*
    left_drive = hardwareMap.dcMotor.get("left_drive");
    right_drive = hardwareMap.dcMotor.get("right_drive");
    back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
    back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
    right_drive.setDirection(DcMotor.Direction.REVERSE);
    back_right_drive.setDirection(DcMotor.Direction.REVERSE);
    Intake = hardwareMap.dcMotor.get("Intake");
    bucket = hardwareMap.servo.get("bucket");
    Slide = hardwareMap.dcMotor.get("Slide");
    Caro = hardwareMap.dcMotor.get("Caro");
    */


    waitForStart();
    while (opModeIsActive() && !isStopRequested()) {


      // drive_encoder(29.045, .5, "fwd");

        /*if(color.red() > 30) {
            drive_encoder(6,.5,"fwd");
        }*/







      telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
      telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
      telemetry.addData("red: ", color.red()); //checking for colors
      telemetry.addData("green: ", color.green()); //checking for colors
      telemetry.addData("blue: ", color.blue()); //checking for colors
      telemetry.addData("alpha:/light ", color.alpha()); //the amount of light is alpha
      telemetry.update();

    }


  }

}
