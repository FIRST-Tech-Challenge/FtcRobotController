package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private Button ctrlProc;
    private Button ctrlIn;
    private Button ctrlOut;
    private DcMotor motor;
    private AsynchronousSleep sleeper = new AsynchronousSleep();
    private boolean waiting = false;

    private enum State {
        STOP, RUNNING, REVERSE;
    }

    private State state = State.STOP;

    public Intake(HardwareMap hardwareMap, ExtendedGamepad extGamepad2) {
        this.ctrlProc = extGamepad2.right_bumper;
        this.ctrlIn = extGamepad2.dpad_left;
        this.ctrlOut = extGamepad2.dpad_right;
        this.motor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public boolean isBusy() {
        return state != State.STOP;
    }

    public boolean isRunning() {
        return state == State.RUNNING;
    }

    public boolean isReverse() {
        return state == State.REVERSE;
    }

    public void run() {
        if(ctrlProc.isBumped() || waiting) {
            sleeper.update();
            if (state == State.RUNNING) {
                state = State.REVERSE;
                motor.setPower(-1);
                sleeper.wait(500);
                waiting = true;
            } else {
                if (sleeper.isReady()) {
                    motor.setPower(0);
                    state = State.STOP;
                    sleeper.reset();
                    waiting = false;
                } else if (!sleeper.isBusy()) {
                    state = State.RUNNING;
                    motor.setPower(1);
                }
            }
        } else if (state == State.STOP) {
            if (ctrlIn.isPressed()) motor.setPower(1);
            else if (ctrlOut.isPressed()) motor.setPower(-1);
            else motor.setPower(0);
        }
    }
}