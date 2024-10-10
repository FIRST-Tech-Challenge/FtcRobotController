package org.firstinspires.ftc.teamcode.tatooine.SubSystem;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

public class Intake  {
    CRServo intake = null;
    CheckAlliance alliance;
    ColorSensorOur colorSensorOur;
    Telemetry telemetry;
    private double power = 0;
    private final double INTAKE_SPEED = 1;
    private final double OUTTAKE_SPEED = 1;

    private boolean buttonPressed = false;

    private boolean isRed =false;
    private boolean isSpasimen;


    public Intake(OpMode opMode, boolean isRed) {
        this.alliance = alliance;
        telemetry = opMode.telemetry;
        colorSensorOur = new ColorSensorOur(opMode);
        intake = opMode.hardwareMap.get(CRServo.class, "intake");
        this.isRed = isRed;
        telemetry.addData("isred",isRed);
        telemetry.update();
    }
    public Action intake(boolean buttonPressed){
        this.buttonPressed = buttonPressed;
        power = INTAKE_SPEED;
        return new SetPowerAction();
    }
    public Action outtake(boolean buttonPressed){
        power = OUTTAKE_SPEED;
        return new SetPowerAction();
    }
    public Action isRightColor(boolean isSpecimen) {
        if (isSpecimen) {
            if (colorSensorOur.getDistance() > 2) {
                power = INTAKE_SPEED;
            } else if (colorSensorOur.getDistance() < 2 && !colorSensorOur.isRightColor(true)) {
                power = OUTTAKE_SPEED;
            }
        } else {
            if (colorSensorOur.getDistance() > 2) {
                power = INTAKE_SPEED;
            } else if (colorSensorOur.getDistance() < 2 && !colorSensorOur.isRightColor(false)) {
                power = OUTTAKE_SPEED;
            }
        }
        telemetry.addData("Color",colorSensorOur.isRightColor(true));
        telemetry.addData("Distance", colorSensorOur.getDistance());
        telemetry.addData("isSpecimen", isSpecimen);
        telemetry.addData("Speed", power);

    return new SetPowerAction();
    }

    public class SetPowerAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(power);
            return colorSensorOur.getDistance()>2 && !colorSensorOur.isRightColor(isSpasimen) ;
        }
    }
}