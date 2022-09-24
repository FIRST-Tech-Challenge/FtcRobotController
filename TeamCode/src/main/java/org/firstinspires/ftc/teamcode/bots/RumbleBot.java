package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RumbleBot extends LEDBot{

    public Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    public boolean vibrate = false;
    boolean rumbleCheck = true;

    public RumbleBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();
    }

    public void rumble() {
        if (distanceBox < 8 && rumbleCheck) {
            vibrate = true;
            rumbleCheck = false;
        } else if (distanceBox >= 8) {
            vibrate = false;
            rumbleCheck = true;
        } else {
            vibrate = false;
            rumbleCheck = false;
        }
    }

    protected void onTick(){
        rumble();
        super.onTick();
    }
}
