package org.firstinspires.ftc.teamcode.core.robot.tools.headless;

import android.util.Pair;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerLift;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.types.api.RunListenerIndefinitelyEvent;
import org.firstinspires.ftc.teamcode.core.thread.types.impl.TimedEvent;

import java.util.concurrent.atomic.AtomicBoolean;

import androidx.annotation.NonNull;

public class AutoLift {

    // INTAKING motor 0 , arm 0.76
    // GOING OUT OF DANGER 0.7
    // OUT OF DANGER motor 1375
    // TOP motor 2800 arm 0.3 - 5 1/4
    // MIDDLE motor 1850 arm 0.3 - 5 1/4 from back to rim
    // BOTTOM motor 1375 arm 0.25 - 5 1/4 from back to rim

    @SuppressWarnings("unused")
    public enum Positions {
        INTAKING(0, 0.7D),
        SAFE(1375, 0.7D),
        TOP(2880, 0.3D),
        MIDDLE(1850, 0.3D),
        BOTTOM(1375, 0.25D);

        public final int motorPos;
        public final double armPos;

        Positions(int motorPos, double armPos) {
            this.motorPos = motorPos;
            this.armPos = armPos;
        }
    }

    private enum MovementStates { // switch this to a bool if you have time
        NO_MOVEMENT,
        MOTOR_MOVEMENT
    }

    private final DcMotor liftMotor;
    private final Servo armServo;
    // private final DigitalChannel bottomSensor;
    private final AtomicBoolean dumping = new AtomicBoolean(false);
    private final AtomicBoolean dumpingEventRunning = new AtomicBoolean(false);
    private final EventThread eventThread;
    private TimedEvent event;
    private Positions position = Positions.INTAKING;
    private Positions lastPosition = Positions.INTAKING;
    private MovementStates state = MovementStates.NO_MOVEMENT;
    // fix this later
    /**
     * @param eventThread local eventThread instance
     * @param map         local hardwareMap instance
     */
    public AutoLift(EventThread eventThread, @NonNull HardwareMap map) {
        liftMotor = map.get(DcMotor.class,"liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armServo = map.get(Servo.class,"armServo");
        this.eventThread = eventThread;
    }

    public void goTo(@NonNull Positions positions) {
        this.position = Positions.INTAKING;
    }

    public void update() {
        if (position != lastPosition) state = MovementStates.NO_MOVEMENT;
        switch (state) {
            case NO_MOVEMENT:
                if (position != lastPosition) {
                    armServo.setPosition(position.armPos);
                    Positions ogposition = position;
                    eventThread.addEvent(new TimedEvent(() -> {
                        if (ogposition == position) {
                            state = MovementStates.MOTOR_MOVEMENT;
                        }
                    }, 800));
                } else if (position == Positions.INTAKING) {
                    armServo.setPosition(0.76D);
                }
                break;
            case MOTOR_MOVEMENT:
                liftMotor.setTargetPosition(position.motorPos);
        }
        lastPosition = position;
    }
}
