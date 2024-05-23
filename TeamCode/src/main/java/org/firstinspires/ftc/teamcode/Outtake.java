package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Outtake {

    public static final double P_CONSTANT = 0.001;
    DcMotor lsFront, lsBack;

    public Outtake(Robot robot) {
        lsFront = robot.lsFront;
        lsBack = robot.lsBack;
    }

    public double lsParallelPowerPSequence(GenericState conditionState, OuttakeState state, double maxPower) {
        if (conditionState.isDone()) {
            if (!state.isDone() && state.getOpMode().opModeIsActive()) {
                return Range.clip(state.getError() * P_CONSTANT, -maxPower, maxPower);
            }
        }
        return 0;
    }
}
