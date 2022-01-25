package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoGrabber;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.opmodes.util.GoodTriggerReader;

/**
 * lift and arm
 */
public class ControllerLift extends AutoLift {
    private final GoodTriggerReader leftTrigger, rightTrigger;
    private final ButtonReader leftBumper, rightBumper, downDPad, rightDPad;

    /**
     * @param eventThread local eventThread instance
     * @param map         local hardwareMap instance
     */
    public ControllerLift(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad, AutoGrabber grabber) {
        super(eventThread, map, grabber);
        leftTrigger = new GoodTriggerReader(toolGamepad, GamepadKeys.Trigger.LEFT_TRIGGER);
        rightTrigger = new GoodTriggerReader(toolGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);
        leftBumper = new ButtonReader(toolGamepad, GamepadKeys.Button.LEFT_BUMPER);
        rightBumper = new ButtonReader(toolGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        downDPad = new ButtonReader(toolGamepad, GamepadKeys.Button.DPAD_DOWN);
        rightDPad = new ButtonReader(toolGamepad, GamepadKeys.Button.DPAD_RIGHT);
    }

    public void init() {
        Thread thread = new Thread(() -> {
            while (!eventThread.isInterrupted()) {
                leftTrigger.readValue();
                rightTrigger.readValue();
                leftBumper.readValue();
                rightBumper.readValue();
                downDPad.readValue();
                rightDPad.readValue();
                if (leftTrigger.wasJustReleased()) setPosition(Positions.TOP);
                else if (rightTrigger.wasJustReleased()) setPosition(Positions.SAFE);
                else if (leftBumper.wasJustReleased()) setPosition(Positions.TSE);
                else if (rightTrigger.wasJustReleased()) setPosition(Positions.INTAKING);
                else if (downDPad.wasJustReleased()) setPosition(Positions.BOTTOM);
                else if (rightDPad.wasJustReleased()) setPosition(Positions.DUMPTSE);
            }
        });
        thread.setPriority(5);
        thread.start();
    }
}
