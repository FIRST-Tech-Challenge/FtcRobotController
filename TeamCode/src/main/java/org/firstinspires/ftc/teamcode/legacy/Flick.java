package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

public class Flick extends LinearOpMode {

  private Servo servo = null;
  private double max = .5; // Maximum rotational position, NOT USED, overridden by servo "chain" in teleop
  private double min = .8; // Minimum rotational position

  public Flick (Servo s) {
    servo = s;
  }

  public void up() {
    servo.setPosition(max);
  }
  public void down() {
    servo.setPosition(min);
  }
  
  public void setPos(double goal) {
    double error = 0.09;
    double increment = 0.06;
    double pos1 = servo.getPosition();
    if (pos1 > goal) {
      pos1 -= increment;
    } else if (pos1 < goal){
      pos1 += increment;
    } else if (pos1 < goal - error && pos1 > goal + error) { //also these signs are backwards
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
