package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFColorSensor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLEDStrip;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Config
public class Magazine {
//  private RFLEDStrip blinkin;
//  private LED rf, rb, lf, lb;
  private RFColorSensor colorSensor1, colorSensor2;

  public static int pixels = 0;
  public static double twoPixelTime = 0.0;
  double dist1=100,dist2=100;

  public Magazine() {
//    rf = op.hardwareMap.get(LED.class, "rf");
//    rb = op.hardwareMap.get(LED.class, "rb");
//    lf = op.hardwareMap.get(LED.class, "lf");
//    lb = op.hardwareMap.get(LED.class, "lb");

    colorSensor1 = new RFColorSensor("colorSensor");
    colorSensor2 = new RFColorSensor("colorSensor2");
//    blinkin = new RFLEDStrip("blinkin");
    pixels=0;
  }

  public enum MagStates {
    FRONT(false),
    BACK(false);

    private boolean state;

    MagStates(boolean p_state) {
      this.state = p_state;
    }

    void toggle() {
      this.state = !this.state;
    }

    void setState(boolean c_state) {
      this.state = c_state;
    }

    public boolean getState() {
      return this.state;
    }
  }

  public double frontdist() {
    return colorSensor1.getDist();
  }

  public double backdist() {
    return colorSensor2.getDist();
  }

  public void updateSensors() {
    dist1 = colorSensor1.getDist();
    dist2 = colorSensor2.getDist();
    pixels=0;
    if (dist1 < 4) {
      MagStates.FRONT.setState(true);
      pixels++;
    } else if (dist1 > 4) {
      MagStates.FRONT.setState(false);
    }
    if (dist2 < 0.9) {
      MagStates.BACK.setState(true);
      pixels++;
    } else if (dist2 > 0.9) {
      MagStates.BACK.setState(false);
    }

    if(pixels!=1 && dist2>1){
      twoPixelTime = BasicRobot.time;
    }
    if(dist2<1.3&&pixels==0){
      pixels=1;
    }
    LOGGER.log("front | back dist: " + dist1 + " | " + dist2);
    LOGGER.log(
            "front | back state: " + MagStates.FRONT.getState() + " | " + MagStates.BACK.getState());
    LOGGER.log("# Pixels: " + getPixels());
  }

  public boolean solidTwoPixels(){
    if (Magazine.pixels == 1) {
      return BasicRobot.time - twoPixelTime > 0.35;
    }
    else{
      return false;
    }
  }



//  public void updateLEDs() {
//    if (Arm.ArmStates.HOVER.state && Claw.clawStates.GRAB.state) {
//      rf.enable(true);
//      rb.enable(true);
//      lf.enable(true);
//      lb.enable(true);
//    } else if (pixels == 0) {
//      rf.enable(false);
//      rb.enable(false);
//      lf.enable(false);
//      lb.enable(false);
//    } else if (pixels == 1) {
//      rf.enable(false);
//      rb.enable(false);
//      lf.enable(false);
//      lb.enable(false);
//    } else if (pixels == 2) {
//      rf.enable(true);
//      rb.enable(false);
//      lf.enable(true);
//      lb.enable(false);
//    }
//  }

  public int getPixels() {
    return pixels;
  }

  public void update() {
    LOGGER.log("front | back dist: " + dist1 + " | " + dist2);
    LOGGER.log(
        "front | back state: " + MagStates.FRONT.getState() + " | " + MagStates.BACK.getState());
    LOGGER.log("# Pixels: " + getPixels());

    if (isTeleop) {
//      updateLEDs();
      updateSensors();
    }
  }
}
