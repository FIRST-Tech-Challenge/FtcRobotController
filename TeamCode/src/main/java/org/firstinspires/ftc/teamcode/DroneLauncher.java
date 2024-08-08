package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {

    DcMotor wheel;
    Servo engageServo;

    public static final double P_CONSTANT = 0.01;

    public DroneLauncher(Robot robot) {
        robot.planeLauncher = robot.hardwareMap.dcMotor.get("planeLauncher");
        this.wheel = robot.planeLauncher;
        this.engageServo = robot.planeLauncherServo;
    }

    public double wheelParallelPowerPSequence(GenericState conditionState, DroneLauncherState state) {
        if (conditionState.isDone()) {
            if (!state.isDone() && state.getOpMode().opModeIsActive()) {
                return P_CONSTANT * state.getError();
            }
        }

        return 0;
    }

    public void setState (double currentTicks, double currentTargetTicks, DroneLauncherState state) {
        state.setCurrentTargetTicks(currentTargetTicks);
        state.setCurrentTicks(currentTicks);
    }


}
