
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

   public void setRed() {
    rgbLight.setPosition(0.28);
}

public void setGreen() {
    rgbLight.setPosition(0.5);
}

public void setBlue() {
    rgbLight.setPosition(0.611);
}

public void setPurple() {
    rgbLight.setPosition(0.7);
}

public void turnOff() {
    rgbLight.setPosition(0.0);
}


      while (opModeIsActive()) {

    // CROSS = RED
    if (gamepad1.a) {
        lights.setRed();
    }

    // CIRCLE = GREEN
    if (gamepad1.b) {
        lights.setGreen();
    }

    // SQUARE = BLUE
    if (gamepad1.x) {
        lights.setBlue();
    }

    // TRIANGLE = PURPLE
    if (gamepad1.y) {
        lights.setPurple();
    }

    // D-PAD DOWN = OFF
    if (gamepad1.dpad_down) {
        lights.turnOff();
    }
            }
        }
    }
}