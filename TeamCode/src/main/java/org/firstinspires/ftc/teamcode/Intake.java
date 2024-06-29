package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    DcMotor wheelMotor;

    public static final double P_CONSTANT = 0.01;

    public Intake(Robot robot) {
        this.wheelMotor = robot.intakeMotor;
    }

    public double intakeToTicksParallelPowerPSequence(GenericState conditionState, IntakeState state) {
        if (conditionState.isDone()) {
            if(!state.isDone() && state.getOpMode().opModeIsActive()) {
                return P_CONSTANT * state.getError();
            }
        }

        return 0;
    }

    public void setState (double currentTicks, double currentTargetTicks, IntakeState state) {
        state.setCurrentTicks(currentTicks);
        state.setCurrentTargetTicks(currentTargetTicks);
    }

}