package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Outtake {

    public static final double P_CONSTANT = 0.001;
    DcMotor lsFront, lsBack;
    private double CLAMP_OPEN_POS = 0.471;
    private double CLAMP_CLOSE_POS = 0.57;
    private double TRAY_INTAKE_POS = 0.3;
    private double TRAY_OUTTAKE_POS = 0;

    public Outtake(Robot robot) {
        lsFront = robot.lsFront;
        lsBack = robot.lsBack;
    }

    public double lsToTicksParallelPowerPSequence(GenericState conditionState, OuttakeState state, double maxPower) {
        if (conditionState.isDone()) {
            if (!state.isDone() && state.getOpMode().opModeIsActive()) {
                return Range.clip(state.getError() * P_CONSTANT, -maxPower, maxPower);
            }
        }
        return 0;
    }

    public void setState (double currentTicks, double currentTargetTicks, OuttakeState state) {
        state.setCurrentTicks(currentTicks);
        state.setCurrentTargetTicks(currentTargetTicks);
    }

    public double getCLAMP_OPEN_POS () {
        return CLAMP_OPEN_POS;
    }

    public double getCLAMP_CLOSE_POS() {
        return CLAMP_CLOSE_POS;
    }

    public double getTRAY_INTAKE_POS() {
        return TRAY_INTAKE_POS;
    }

    public double getTRAY_OUTTAKE_POS() {
        return TRAY_OUTTAKE_POS;
    }
}
