package org.firstinspires.ftc.teamcode.machanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ProgrammingBoard2 {
  private DigitalChannel touchSensor;
  
  public void init (HardwareMap hwMap) {
    touchSensor = hwMap.get(DigitalChennel.class, "touch_sensor");
    touchSensor.setMode(DigitalChennel.Mode.INPUT);
  }
  
  publicboolean isTouchSensorPressed() {
    return !touchSensor.getState();
  }
}
