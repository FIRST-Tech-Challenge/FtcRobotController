package org.firstinspires.ftc.teamcode.core.robot.tools.headless;

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
import org.firstinspires.ftc.teamcode.core.thread.types.impl.RunWhenOutputChangedOnceEvent;
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
        INTAKING(0, 0.76D, false),
        SAFE(1375, 0.7D, false),
        TOP(2880, 0.3D, true),
        MIDDLE(1850, 0.3D, true),
        BOTTOM(1375, 0.25D, true);

        public final double armPos;
        public final int motorPos;
        public final boolean dumper;
        
        Positions(int motorPos, double armPos, boolean dumper) {
            this.motorPos = motorPos;
            this.armPos = armPos;
            this.dumper = dumper;
        }
    }

    protected enum MovementStates { // switch this to a bool if you have time
        NO_MOVEMENT,
        MOVING,
        SERVO_MOVEMENT
    }

    protected final DcMotor liftMotor;
    protected final Servo armServo;
    // protected final DigitalChannel bottomSensor;
    protected final AtomicBoolean dumping = new AtomicBoolean(false);
    protected final AtomicBoolean dumpingEventRunning = new AtomicBoolean(false);
    protected final EventThread eventThread;
    protected TimedEvent event;
    protected Positions position = Positions.INTAKING;
    protected Positions lastPosition = Positions.INTAKING;
    protected MovementStates state = MovementStates.NO_MOVEMENT;
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

    public void setPosition(@NonNull Positions position) {
        this.position = position;
    }

    public void blockingSetPosition(@NonNull Positions position) {
        setPosition(position);
        //insert some funny code that blocks until it has moved to position, will be very useful for finian burkard auto
    }

    protected boolean eval() {
        return (liftMotor.getCurrentPosition() >= position.motorPos - 10 && liftMotor.getCurrentPosition() <= position.motorPos + 10);
    }


    /*
    set to 0.7
    go to motor position
    set servo to servo position
    if not dumper BREAK
    wait 800 ms
    go to motor 1375 and servo 0.7
    BREAK
     */

    public void update() {
        if (position != lastPosition) state = MovementStates.NO_MOVEMENT;
        switch (state) {
            case NO_MOVEMENT:
                armServo.setPosition(0.7D);
                liftMotor.setTargetPosition(position.motorPos);
                state = MovementStates.MOVING;
                eventThread.addEvent(new RunWhenOutputChangedOnceEvent(() -> {
                    state = MovementStates.SERVO_MOVEMENT
                },this::eval));
                break;
            case SERVO_MOVEMENT:
                armServo.setPosition(position.armPos);
                Positions ogposition = position;
                eventThread.addEvent(new TimedEvent(() -> {
                    if (ogposition == position) {
                        state = MovementStates.MOTOR_MOVEMENT;
                    }
                }, 800));
        }
        lastPosition = position;
    }
}
