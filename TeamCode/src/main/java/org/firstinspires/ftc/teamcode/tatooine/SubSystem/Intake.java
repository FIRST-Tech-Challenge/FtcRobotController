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
    ColorSensorOur colorSensorOur = null;
    private double power = 0;
    private boolean buttonPressed = false;


    public Intake(HardwareMap hardwareMap) {
        this.alliance = alliance;
        colorSensorOur = new ColorSensorOur(hardwareMap);
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
    public Action isRightColorFor(boolean isSpecimen) {
        if (isSpecimen && !colorSensorOur.isRightColorForSpecimen()){
            power = -1;
        }
        else if (!isSpecimen && !colorSensorOur.isRightColorForSample()){
            power = -1;
        }
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