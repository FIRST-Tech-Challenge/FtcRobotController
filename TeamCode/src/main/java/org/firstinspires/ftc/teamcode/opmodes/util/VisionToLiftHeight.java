package org.firstinspires.ftc.teamcode.opmodes.util;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;

public class VisionToLiftHeight {
    public static AutoLift.Positions getPosition(int input) {
        return input == 1 ? AutoLift.Positions.BOTTOM :
                input == 2 ? AutoLift.Positions.MIDDLE : AutoLift.Positions.AUTOTOP;
    }
}
