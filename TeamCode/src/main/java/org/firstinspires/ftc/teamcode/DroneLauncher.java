package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {

    DcMotor wheel;
    Servo engageServo;

    public DroneLauncher(Robot robot) {
        this.wheel = robot.planeLauncher;
        this.engageServo = robot.planeLauncherServo;
    }

    public int testDroneMotorSequencePower(GenericState conditionState, DroneLauncherState state) {
        if (!state.isDone() && state.getOpMode().opModeIsActive()) {
            return 500;
        }
        return 0;
    }

    public void setState (double currentTicks, double currentTargetTicks, DroneLauncherState state) {
        state.setCurrentTicks(currentTicks);
        state.setCurrentTargetTicks(currentTargetTicks);
    }


}
