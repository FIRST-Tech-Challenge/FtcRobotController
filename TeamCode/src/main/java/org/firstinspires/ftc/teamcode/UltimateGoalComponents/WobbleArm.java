package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;

public class WobbleArm extends RobotComponent {


    public WobbleArm(robotBase BASE) {
        super(BASE);
    }

    public enum POSITION {OPEN_POSITION, CLOSE_POSITION}

    @Override
    public void stop() {

    }
}
