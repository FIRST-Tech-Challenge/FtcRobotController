package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TelePivotController {

    private static final double p_INCREMENT = 0.001;
    private Servo p_pivot;

    public TelePivotController(Servo pivot) {
        p_pivot = pivot;

        p_pivot.setPosition(1);
    }

    public void setPosition (double position) {
        p_pivot.setPosition(position);
    }

    public void update (double power) {
        p_pivot.setPosition(p_pivot.getPosition() + power * p_INCREMENT);
    }
}
