package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {

    public enum State {TRANSFER, IDLE}
    private final DcMotorEx transfer;
    private State state = State.IDLE;

    public Transfer(HardwareMap hardwareMap) {
        this(hardwareMap, "transfer");
    }

    public Transfer(HardwareMap hardwareMap, String name) {
        transfer = hardwareMap.get(DcMotorEx.class, name);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setState(State newState) {
        state = newState;
        switch (newState) {
            case TRANSFER:
                transfer.setPower(1.0);
                break;
            case IDLE:
            default:
                transfer.setPower(0);
                break;
        }
    }

    public State getState() {
        return state;
    }

    public Command transfer() {
        return instant(() -> setState(State.TRANSFER)).requiring(this);
    }

    public Command idle() {
        return instant(() -> setState(State.IDLE)).requiring(this);
    }
}
