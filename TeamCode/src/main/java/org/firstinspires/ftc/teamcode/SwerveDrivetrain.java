package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Vector;


@TeleOp(name = "SwerveDrive", group = "Driving")
public class SwerveDrivetrain extends OpMode {


  DcMotorEx fr;
  DcMotorEx fl;
  Servo frs;
  Servo fls;
  int targetPosition = 0;



  public static boolean robotCentric = false;

    public void init() {
      fr = hardwareMap.get(DcMotorEx.class, "fr");
      fl = hardwareMap.get(DcMotorEx.class, "fl");
      frs = hardwareMap.get(Servo.class, "frs");
      fls = hardwareMap.get(Servo.class, "fls");
      frs.setPosition(0.5);
      fls.setPosition(0.5);
      fr.setTargetPosition(targetPosition);
      fl.setTargetPosition(targetPosition);
      fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void init_loop() {
      //INIT LOOP
      //We never put anything here for motors/servos
      //as its against the rules for the bots to move AT ALL
      //in the initialization phase of the Driver Controlled Period.
      //For testing purposes, obviously yes you can put stuff here.

    }

    @Override
    public void loop() {
      double power = Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y);
      double angle = 2 - Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x)/Math.PI-1;

      frs.setPosition(angle%1);
      fls.setPosition(angle%1);
      fr.setTargetPosition(targetPosition);
      fl.setTargetPosition(targetPosition);
      fr.setPower(.5);
      fl.setPower(.5);

      telemetry.addData("Angle", angle);

      if (angle < 1) {
        targetPosition -= (int) power;
      }
      else {
        targetPosition += (int) power;
      }

      telemetry.update();
    }
}
