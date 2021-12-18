package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TeleElevatorController {

    private static final double p_EPSILON = 0.001;
    private final int p_RANGE = 5000;
    private int p_onePosition;
    private DcMotor p_elevator;

    public TeleElevatorController(DcMotor elevator) {
        p_elevator = elevator;

        p_onePosition = p_elevator.getCurrentPosition() + p_RANGE;
    }

    public boolean ascend (double power) {
        if (p_elevator.getCurrentPosition() >= p_onePosition) {
            p_elevator.setPower(0);
            return false;
        }
        p_elevator.setPower(getPower(power));
        return true;
    }

    public boolean descend (double power, boolean atBottom) {
        if (atBottom) {
            p_elevator.setPower(0);
            setBounds();
            return false;
        }
        p_elevator.setPower(-1 * getPower(power));
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

    private void setBounds () {
        p_onePosition = p_elevator.getCurrentPosition() + p_RANGE;
    }
}
