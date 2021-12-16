package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmController {

    private DcMotor p_arm;
    private final int p_RANGE = 50;
    private final int p_ZERO_POSITION, p_ONE_POSITION;

    public ArmController(DcMotor arm) {
        p_arm = arm;

        p_ZERO_POSITION = arm.getCurrentPosition();
        p_ONE_POSITION = p_ZERO_POSITION + p_RANGE;
    }

    public boolean setPosition (double position) {
        if (position > 1 || position < 0)
            return false;

        p_arm.setTargetPosition( (int) (
                position * p_ZERO_POSITION
                + (1 - position) * p_ONE_POSITION
        ));
        return true;
    }
}
