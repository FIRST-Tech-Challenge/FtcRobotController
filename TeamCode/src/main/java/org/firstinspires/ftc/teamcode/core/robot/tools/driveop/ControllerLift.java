package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.core.thread.types.impl.RunWhenOutputChangedIndefinitelyEvent;

/**
 * lift and arm
 */
public class ControllerLift extends AutoLift {
    private final TriggerReader leftReader;
    private final TriggerReader rightReader;
    private final GamepadEx toolGamepad;
    /**
     * @param eventThread local eventThread instance
     * @param map         local hardwareMap instance
     */
    public ControllerLift(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad) {
        super(eventThread, map);
        this.toolGamepad = toolGamepad;
        leftReader = new TriggerReader(toolGamepad, GamepadKeys.Trigger.LEFT_TRIGGER);
        rightReader = new TriggerReader(toolGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    public void init() {
        eventThread.addEvent(new RunWhenOutputChangedIndefinitelyEvent(() -> setPosition(Positions.TOP), () -> {
            leftReader.readValue();
            return leftReader.wasJustReleased();
        }));
        eventThread.addEvent(new RunWhenOutputChangedIndefinitelyEvent(() -> setPosition(Positions.SAFE), () -> {
            rightReader.readValue();
            return rightReader.wasJustReleased();
        }));
    }

    @Override
    public void update() {
        super.update();
        if (position == Positions.SAFE && state == null) {
            if (liftMotor.getCurrentPosition() >= 1375) { // PUT LIMTI SWITHC STUFF HERE
                liftMotor.setPower(toolGamepad.getLeftY());
            } else {
                liftMotor.setPower(0);
            }
        }
    }
}
