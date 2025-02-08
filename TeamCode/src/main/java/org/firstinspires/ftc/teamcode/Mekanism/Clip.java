package org.firstinspires.ftc.teamcode.Mekanism;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Clip {

  LinearOpMode myOp;

  private Servo
      ramp1,
      ramp2,
      funnel;

  private double rampPos = 0, funnelPos = 0;

  private boolean clamped = false;


  public Clip(LinearOpMode opMode) {

    ramp1 = opMode.hardwareMap.get(Servo.class, "ramp 1");
    ramp2 = opMode.hardwareMap.get(Servo.class, "ramp 2");
    funnel = opMode.hardwareMap.get(Servo.class, "funnel");

    ramp1.setDirection(FORWARD);
    ramp2.setDirection(REVERSE);
    funnel.setDirection(FORWARD);

    ramp1.scaleRange(0, 0.15);
    ramp2.scaleRange(0, 0.15);
    funnel.scaleRange(0, 0.035);
    myOp = opMode;
  }

  public void update() {

    if (clamped) {
      rampPos = 1;
    } else {
      rampPos = 0;
    }

    ramp1.setPosition(rampPos);
    ramp2.setPosition(rampPos);

    funnel.setPosition(funnelPos);
  }


  public void setFunnel(double pos) {
    funnelPos = pos;
  }


  public void clamp() {
    clamped = true;
  }

  public void unclamp() {
    clamped = false;
  }


}
