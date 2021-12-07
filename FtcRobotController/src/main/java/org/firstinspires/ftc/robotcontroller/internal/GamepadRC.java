package org.firstinspires.ftc.robotcontroller.internal;

import android.view.KeyEvent;
import android.view.MotionEvent;

//import com.qualcomm.hardware.logitech.LogitechGamepadF310;
//import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice.


public class GamepadRC {
    public static final LegacyGamepad gamepadRC = new LogitechGamepadF310();

    public synchronized void update(MotionEvent event) {
        gamepadRC.update(event);
    }

    public synchronized void update(KeyEvent event) {
        gamepadRC.update(event);
    }
}
