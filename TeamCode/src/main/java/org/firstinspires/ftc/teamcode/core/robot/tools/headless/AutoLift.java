package org.firstinspires.ftc.teamcode.core.robot.tools.headless;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.old.types.api.RunListenerOnceEvent;
import org.firstinspires.ftc.teamcode.core.thread.old.types.impl.TimedEvent;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

public class AutoLift {
    // 5 1/4 inch from back of robot to rim
    public enum Positions {
        INTAKING(0, 0.76D, false),
        SAFE(703, 0.7D, false),
        TOP(1250, 0.3D, true),
        MIDDLE(946, 0.3D, true),
        BOTTOM(703, 0.25D, true),
        TSE(2230, 0.716D, false),
        SAFETOP(TOP.motorPos, SAFE.armPos, false),
        FUNNYDUMP(TSE.motorPos, 1, true),
        DUMPTSE(TOP.motorPos, FUNNYDUMP.armPos, true),
        AUTOTOP(TOP.motorPos, 0, true);

        public final double armPos;
        public final int motorPos;
        public final boolean dumper;
        
        Positions(int motorPos, double armPos, boolean dumper) {
            this.motorPos = motorPos;
            this.armPos = armPos;
            this.dumper = dumper;
        }
    }

    public enum MovementStates { // switch this to a bool if you have time
        NONE,
        START,
        LIFT_MOVEMENT,
        SERVO_MOVEMENT
    }

    public final DcMotor liftMotor;
    protected final Servo armServo;
    protected final EventThread eventThread;
    protected Positions position = Positions.INTAKING;
    protected Positions lastPosition = position;
    protected MovementStates state = MovementStates.NONE;
    protected AutoGrabber grabber;
    private final ElapsedTime timer = new ElapsedTime();
    // fix this later
    /**
     * @param eventThread local eventThread instance
     * @param map         local hardwareMap instance
     * @param grabber     grabber instance
     */
    public AutoLift(EventThread eventThread, @NonNull HardwareMap map, @Nullable AutoGrabber grabber) {
        this.liftMotor = map.get(DcMotor.class,"liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
        liftMotor.setTargetPosition(Math.abs(liftMotor.getCurrentPosition()));
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1D);
        this.armServo = map.get(Servo.class,"armServo");
        this.eventThread = eventThread;
        this.grabber = grabber;
        this.armServo.setPosition(Positions.INTAKING.armPos);
        final Thread thread = new Thread(() -> {
            while (eventThread.isAlive()) {
                liftMotor.setPower(liftMotor.getTargetPosition() >= liftMotor.getCurrentPosition() ? 1D : 0.75D);
            }
        });
        thread.setPriority(7);
        thread.start();
    }
    /**
     * @param eventThread local eventThread instance
     * @param map         local hardwareMap instance
     */
    public AutoLift(EventThread eventThread, HardwareMap map) {
        this(eventThread, map, null);
    }

    public void setPosition(@NonNull Positions position) {
        this.position = position;
    }

    public Positions getPosition() {
        return position;
    }

    public void blockingSetPosition(@NonNull Positions position) {
        setPosition(position);
        //insert some funny code that blocks until it has moved to position, will be very useful for finian burkard auto
    }

    private boolean liftWaiting = true;

    public void update() {
        if (Objects.requireNonNull(position) != lastPosition) state = MovementStates.START;
        switch (state) {
            case START:
                armServo.setPosition(Positions.SAFE.armPos);
                state = MovementStates.LIFT_MOVEMENT;
                if (lastPosition != null) {
                    liftMotor.setTargetPosition(position.motorPos);
                    liftWaiting = false;
                } else {
                    liftWaiting = true;
                    eventThread.addEvent(new TimedEvent(() -> {
                        liftWaiting = false;
                        liftMotor.setTargetPosition(position.motorPos);
                    }, 500));
                }
                break;
            case LIFT_MOVEMENT:
                if (!liftWaiting) {
                    if (Math.abs(liftMotor.getCurrentPosition() - position.motorPos) <= 18) {
                        armServo.setPosition(position.armPos);
                        if (!position.dumper) state = MovementStates.NONE;
                        else {
                            if (position == Positions.TSE && Objects.nonNull(grabber)) {
                                grabber.open();
                            }
                            timer.reset();
                            state = MovementStates.SERVO_MOVEMENT;
                        }
                    }
                } else if (position == Positions.INTAKING) {
                    eventThread.addEvent(new RunListenerOnceEvent(() -> armServo.setPosition(Positions.INTAKING.armPos)) {
                        @Override
                        public boolean shouldRun() {
                            return liftMotor.getCurrentPosition() <= 5;
                        }
                    });
                    state = MovementStates.NONE;
                }
                break;
            case SERVO_MOVEMENT:
                if (timer.time(TimeUnit.MILLISECONDS) >= (position == Positions.TSE ? 800 : 1400)) {
                    position = Positions.INTAKING;
                    lastPosition = null;
                    return;
                }
                break;
        }
        lastPosition = position;
    }

    public MovementStates getState() {
        return state;
    }
}
