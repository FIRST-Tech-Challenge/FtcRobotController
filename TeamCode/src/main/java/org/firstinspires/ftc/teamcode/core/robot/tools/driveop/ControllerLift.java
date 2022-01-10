package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    private final GoodTriggerReader leftTrigger;
    private final GoodTriggerReader rightTrigger;
    private final ButtonReader leftBumper;
    private final GamepadEx toolGamepad;
    private final DigitalChannel topSensor;

    /**
     * @param eventThread local eventThread instance
     * @param map         local hardwareMap instance
     */
    public ControllerLift(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad, AutoGrabber grabber) {
        super(eventThread, map, grabber);
        this.toolGamepad = toolGamepad;
        leftTrigger = new GoodTriggerReader(toolGamepad, GamepadKeys.Trigger.LEFT_TRIGGER);
        rightTrigger = new GoodTriggerReader(toolGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);
        leftBumper = new ButtonReader(toolGamepad, GamepadKeys.Button.LEFT_BUMPER);
        topSensor = map.get(DigitalChannel.class,"topSensor");
        topSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public void init() {
        Thread thread = new Thread(() -> {
            while (!eventThread.isInterrupted()) {
                leftTrigger.readValue();
                rightTrigger.readValue();
                leftBumper.readValue();
                if (leftTrigger.wasJustReleased()) setPosition(Positions.TOP);
                else if (rightTrigger.wasJustReleased()) setPosition(Positions.SAFE);
                else if (leftBumper.wasJustReleased()) setPosition(Positions.TSE);
            }
        });
        thread.setPriority(5);
        thread.start();
    }
}
