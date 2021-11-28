package org.firstinspires.ftc.teamcode.trajectory;

import static org.firstinspires.ftc.teamcode.core.ROBOT_DATA.*;

public class MovementMotionProfileFactory extends MotionProfileFactory{

    private static final MovementMotionProfileFactory instance = new MovementMotionProfileFactory();

    private MovementMotionProfileFactory() {
        super(ACCEL_TIME, MAX_AUTO_SPEED);
    }

    public static MovementMotionProfileFactory getInstance() {
        return instance;
    }
}
