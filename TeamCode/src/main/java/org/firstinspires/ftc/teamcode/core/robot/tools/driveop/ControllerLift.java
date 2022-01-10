package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.core.thread.types.impl.RunWhenOutputChangedIndefinitelyEvent;
import org.firstinspires.ftc.teamcode.opmodes.util.GoodTriggerReader;

/**
 * lift and arm
 */
public class ControllerLift extends AutoLift {
    private final GoodTriggerReader leftReader;
    private final GoodTriggerReader rightReader;
    private final GamepadEx toolGamepad;
    private final DigitalChannel topSensor;

    /**
     * @param eventThread local eventThread instance
     * @param map         local hardwareMap instance
     */
    public ControllerLift(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad) {
        super(eventThread, map);
        this.toolGamepad = toolGamepad;
        leftReader = new GoodTriggerReader(toolGamepad, GamepadKeys.Trigger.LEFT_TRIGGER);
        rightReader = new GoodTriggerReader(toolGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);
        topSensor = map.get(DigitalChannel.class,"topSensor");
        topSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public void init() {
        Thread thread = new Thread(() -> {
            while (!eventThread.isInterrupted()) {
                leftReader.readValue();
                rightReader.readValue();
                if (leftReader.wasJustReleased()) setPosition(Positions.TOP);
                if (rightReader.wasJustReleased()) setPosition(Positions.SAFE);
            }
        });
        thread.setPriority(5);
        thread.start();
    }

    @Override
    public void update() {
        super.update();
        if (position == Positions.SAFE && state == MovementStates.NONE) {
            if (liftMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (liftMotor.getCurrentPosition() >= 1375 && !topSensor.getState()) {
                // deadzone
                if (Math.abs(toolGamepad.getLeftY()) <= 0.1) {
                    liftMotor.setPower(toolGamepad.getLeftY());
                }
            } else {
                liftMotor.setPower(0);
            }
        } else {
            if (liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(1);
            }
        }
    }
}
