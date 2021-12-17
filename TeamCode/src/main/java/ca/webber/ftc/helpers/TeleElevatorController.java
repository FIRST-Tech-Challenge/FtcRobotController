package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TeleElevatorController {

    private static final double p_EPSILON = 0.001;
    private final int p_RANGE = 10000;
    private final double p_POWER = 0.5;
    private int p_zeroPosition, p_onePosition;
    private DcMotor p_elevator;

    public TeleElevatorController(DcMotor elevator) {
        p_elevator = elevator;

        p_zeroPosition = p_elevator.getCurrentPosition();
        p_onePosition = p_zeroPosition + p_RANGE;
    }

    public boolean ascend () {
        if (p_elevator.getCurrentPosition() >= p_onePosition) {
            p_elevator.setPower(0);
            return false;
        }
        p_elevator.setPower(p_POWER);
        return true;
    }

    public boolean descend () {
        if (p_elevator.getCurrentPosition() <= p_zeroPosition) {
            p_elevator.setPower(0);
            return false;
        }
        p_elevator.setPower(p_POWER * -1);
        return true;
    }

    public void stop () {
        p_elevator.setPower(0);
    }

    private double getPower (double power) {
        if (Math.abs(power) < p_EPSILON) {
            return 0;
        }
        return power;
    }
}
