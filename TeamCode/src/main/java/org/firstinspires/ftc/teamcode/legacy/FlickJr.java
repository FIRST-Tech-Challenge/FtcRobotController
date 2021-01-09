package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

public class FlickJr extends LinearOpMode {

  private Servo servo = null;
  private double max = .3; // Maximum rotational position
  private double min = 1; // Minimum rotational position

  public FlickJr (Servo s) {
    servo = s;
  }

  public void up() {
    servo.setPosition(max);
  }
  public void down() {
    servo.setPosition(min);
  }

  public void setPos(double goal) {
    double error = 0.05;
    double increment = 0.06;
    double pos1 = servo.getPosition();
    if (pos1 > goal) {
      pos1 -= increment;
    } else if (pos1 < goal){
      pos1 += increment;
    } else if (pos1 < goal - error && pos1 > goal + error) {
      pos1 = goal;
    }
    servo.setPosition(pos1);
  }
  
  public double getPos() {
    return servo.getPosition();
  }

  public void runOpMode() {

  }
}
