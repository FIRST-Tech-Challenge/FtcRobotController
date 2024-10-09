package org.firstinspires.ftc.teamcode.tatooine.SubSystem;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

public class Intake  {
    CRServo intake = null;
    CheckAlliance alliance;
    private double power = 0;
    private boolean buttonPressed = false;


    public Intake(HardwareMap hardwareMap, CheckAlliance alliance) {
        this.alliance = alliance;
        intake = hardwareMap.get(CRServo.class, "intake");
    }

    public Action intake(boolean buttonPressed){
        this.buttonPressed = buttonPressed;
        power = 1;
        return new SetPowerAction();
    }
    public Action outtake(boolean buttonPressed){
        power = -1;
        return new SetPowerAction();
    }

    public class SetPowerAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(power);
            return buttonPressed;
        }
    }
}