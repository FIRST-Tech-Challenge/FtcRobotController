package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    DcMotor wheelMotor;

    public Intake(Robot robot) {
        this.wheelMotor = robot.intakeMotor;
    }

    public double intakeTestSequencePower(GenericState conditionState, IntakeState state) {
        if (conditionState.isDone()) {
            if (!state.isDone() && state.getOpMode().opModeIsActive()) {
                return 1000;
            }
        }
        return 0;
    }

    public void setState (double currentTicks, double currentTargetTicks, IntakeState state) {
        state.setCurrentTicks(currentTicks);
        state.setCurrentTargetTicks(currentTargetTicks);
    }

}