package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Outtake {

    public static final double P_CONSTANT = 0.001;
    DcMotor lsFront, lsBack;
    Servo tray;
    Servo clamp;
    LinearOpMode opMode;
    private double CLAMP_OPEN_POS = 0.471;
    private double CLAMP_CLOSE_POS = 0.57;
    private double TRAY_INTAKE_POS = 0.3;
    private double TRAY_OUTTAKE_POS = 0;

    public Outtake(Robot robot) {
        lsFront = robot.lsFront;
        lsBack = robot.lsBack;

        tray = robot.tray;
        clamp = robot.clamp;
        opMode = robot.opMode;
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

    public void trayToIntakePos(boolean blocking) {
        //backup value 0.45
        //delta 0.3
        tray.setPosition(getTRAY_INTAKE_POS());
        if (blocking) {
            opMode.sleep(500);
        }
    }

    public void trayToOuttakePos(boolean blocking) {
        //backup value 0.15
        //delta 0.3
        tray.setPosition(getTRAY_OUTTAKE_POS());
        if (blocking) {
            opMode.sleep(100);
        }
    }

    public void closeClamp(boolean blocking) {
        clamp.setPosition(getCLAMP_CLOSE_POS());
        if (blocking) {
            opMode.sleep(300);
        }
    }

    public void openClamp(boolean wide, boolean auto, boolean blocking) {
        if (wide) {
            if (auto) {
                clamp.setPosition(getCLAMP_OPEN_POS());
            }
            else {
                clamp.setPosition(getCLAMP_OPEN_POS());
            }
        } else {
            clamp.setPosition(0.51);
        }

        if (blocking) {
            opMode.sleep(300);
        }
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
