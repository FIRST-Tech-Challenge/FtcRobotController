package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TelePivotController {

    private static final double p_EPSILON = 0.001;
    private final int p_RANGE = 50;
    private int p_zeroPosition, p_onePosition;
    private DcMotor p_pivot;

    public TelePivotController(DcMotor pivot) {
        p_pivot = pivot;

        p_zeroPosition = p_pivot.getCurrentPosition();
        p_onePosition = p_zeroPosition + p_RANGE;
    }

    public void updatePositions () {
        p_zeroPosition = p_pivot.getCurrentPosition();
        p_onePosition = p_zeroPosition + p_RANGE;
    }

    public boolean update (double power) {
        power = getPower(power);

        if (
                power > 0 && p_pivot.getCurrentPosition() >= p_onePosition ||
                power < 0 && p_pivot.getCurrentPosition() <= p_zeroPosition
        ) {
            p_pivot.setPower(0);
            return false;
        } else {
            p_pivot.setPower(power);
            return true;
        }
    }

    private double getPower (double power) {
        if (Math.abs(power) < p_EPSILON) {
            return 0;
        }
        return power;
    }
}
