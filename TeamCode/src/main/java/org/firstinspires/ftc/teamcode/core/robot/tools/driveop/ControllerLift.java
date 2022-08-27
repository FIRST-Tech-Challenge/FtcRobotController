package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoGrabber;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.opmodes.util.GoodTriggerReader;

/**
 * lift and arm
 */
public class ControllerLift extends AutoLift {
    private final GoodTriggerReader leftTrigger, rightTrigger;
    private final ButtonReader leftBumper, rightBumper, downDPad, rightDPad, startButton, leftDPad, upDPad;

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
        upDPad = new ButtonReader(toolGamepad, GamepadKeys.Button.DPAD_UP);
        startButton = new ButtonReader(toolGamepad, GamepadKeys.Button.START);
        leftDPad = new ButtonReader(toolGamepad, GamepadKeys.Button.DPAD_LEFT);
    }
    @Override
    public void update() {
        leftTrigger.readValue();
        rightTrigger.readValue();
        leftBumper.readValue();
        downDPad.readValue();
        rightDPad.readValue();
        leftDPad.readValue();
        upDPad.readValue();
        if (leftTrigger.wasJustReleased()) setPosition(Positions.TOP);
        else if (rightTrigger.wasJustReleased()) setPosition(Positions.SAFE);
        else if (leftBumper.wasJustReleased()) setPosition(Positions.TSE);
        else if (downDPad.wasJustReleased()) setPosition(Positions.BOTTOM);
        else if (rightDPad.wasJustReleased()) setPosition(Positions.DUMPTSE);
        else if (rightBumper.wasJustPressed()) setPosition(Positions.INTAKING);
        else if (leftDPad.wasJustReleased()) setPosition(Positions.FUNNYDUMP);
        else if (upDPad.wasJustReleased()) setPosition(Positions.SAFETOP);
        super.update();
    }
}
