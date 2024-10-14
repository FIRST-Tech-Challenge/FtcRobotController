package org.firstinspires.ftc.teamcode.tatooine.SubSystem;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

public class Intake  {
    CRServo intake = null;
    CheckAlliance alliance;
    ColorSensorOur colorSensorOur;
    Telemetry telemetry;
    private double power = 0;
    private final double INTAKE_SPEED = 1;
    private final double OUTTAKE_SPEED = -INTAKE_SPEED;

    ElapsedTime timer = new ElapsedTime();
    boolean outtaking = false;

    private boolean buttonPressed = false;

    private boolean isRed =false;
    private boolean isSpecimen;

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
    public Action intakeByColor(boolean isSpecimen) {
        this.isSpecimen = isSpecimen;
        telemetry.addData("Color",colorSensorOur.isRightColor(true));
        telemetry.addData("Distance", colorSensorOur.getDistance());
        telemetry.addData("isSpecimen", isSpecimen);
        telemetry.addData("Speed", power);

    return new IntakeByColor();
    }


    public class SetPowerAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(power);
            return false;
        }
    }
    public class IntakeByColor implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean colorCheek = colorSensorOur.isRightColor(isSpecimen);
            double dis = colorSensorOur.getDistance();
            boolean isIn = dis < 2;

            if (!isIn && !outtaking){
                intake.setPower(INTAKE_SPEED);
            }else if (colorCheek){
                intake.setPower(0);
            }
            else {
                intake.setPower(OUTTAKE_SPEED);
                if (!outtaking){
                    timer.reset();
                    outtaking = true;

                }
                else {outtaking = timer.milliseconds() < 750;}


            }

           return (!isIn && !colorCheek);
        }
    }
}