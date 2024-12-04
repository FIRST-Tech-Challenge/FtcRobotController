package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.PID.OpMode;
import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.intoTheDeep.Teleop;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeReadyAction extends Action {
    Teleop teleop = new Teleop();
    OpMode linearOpmode;
    OpModeUtilities opModeUtilities = new OpModeUtilities(teleop.hardwareMap, linearOpmode, teleop.telemetry);
    Intake intake;
    IntakePivotAutoAction intakePivotAutoAction;
    IntakeDoorAutoAction intakeDoorAutoAction;
    public IntakeReadyAction() {
        intake = new Intake(opModeUtilities);
        intakeDoorAutoAction = new IntakeDoorAutoAction(intake, 0.15);
        intakePivotAutoAction = new IntakePivotAutoAction(intake, 0.0);
    }
    @Override
    public boolean checkDoneCondition() {
        return false;
    }
}
