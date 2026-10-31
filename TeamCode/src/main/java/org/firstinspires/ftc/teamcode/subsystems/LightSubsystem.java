
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

   public class LightSubsystem {

    private Servo rgbLight;

    private static final double OFF = 0.0;
    private static final double RED = 0.28;
    private static final double GREEN = 0.5;

    public LightSubsystem(HardwareMap hardwareMap) {

    rgbLight = hardwareMap.get(Servo.class, "shooterRGB");

    rgbLight.setPosition(OFF);
    }
    public void update(boolean shooterOn, boolean shooterReady) {

    if (!shooterOn) {
        rgbLight.setPosition(OFF);
    }
    else if (shooterReady) {
        rgbLight.setPosition(GREEN);
    }
    else {
        rgbLight.setPosition(RED);
    }
    }
    public void setGreen(){
        rgbLight.setPosition(GREEN);
    }
   }