package org.firstinspires.ftc.teamcode.core.robot.tools.headless;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.types.impl.TimedEvent;

import androidx.annotation.NonNull;

public class AutoLift {

    // 5 1/4 inch from back of robot to rim

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
    // fix this later
    /**
     * @param eventThread local eventThread instance
     * @param map         local hardwareMap instance
     */
    public AutoLift(EventThread eventThread, @NonNull HardwareMap map) {
        liftMotor = map.get(DcMotor.class,"liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
        liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
        armServo = map.get(Servo.class,"armServo");
        this.eventThread = eventThread;
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

    private boolean dumpWaiting = true;

    public void update() {
        if (position != lastPosition) state = MovementStates.START;
        switch (state) {
            case START:
                armServo.setPosition(0.7D);
                liftMotor.setTargetPosition(position.motorPos);
                state = MovementStates.LIFT_MOVEMENT;
                break;
            case LIFT_MOVEMENT:
                final double motorPos = liftMotor.getCurrentPosition();
                if (motorPos >= position.motorPos - 10 && motorPos <= position.motorPos + 10) {
                    armServo.setPosition(position.armPos);
                    if (!position.dumper) state = MovementStates.NONE;
                    else {
                        dumpWaiting = true;
                        eventThread.addEvent(new TimedEvent(() -> dumpWaiting = false, 1400));
                        state = MovementStates.SERVO_MOVEMENT;
                    }
                }
                break;
            case SERVO_MOVEMENT:
                if (!dumpWaiting) {
                    position = Positions.INTAKING;
                    return;
                }
                break;
        }
        lastPosition = position;
    }
}
