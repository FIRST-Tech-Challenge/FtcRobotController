package org.firstinspires.ftc.teamcode.tatooine.SubSystem;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

public class Intake {

    //add the variables
    private final double INTAKE_SPEED = 1;
    private final double OUTTAKE_SPEED = -INTAKE_SPEED;
    CRServo intake = null;
    CheckAlliance alliance;
    ColorSensorOur colorSensorOur;
    Telemetry telemetry;
    ElapsedTime timer = new ElapsedTime();
    boolean outtaking = false;
    private double power = 0;
    private boolean buttonPressed = false;

    private boolean isRed = false;
    private boolean isSpecimen;

    //intake constructor
    public Intake(OpMode opMode, boolean isRed) {
        telemetry = opMode.telemetry;
        colorSensorOur = new ColorSensorOur(opMode);
        intake = opMode.hardwareMap.get(CRServo.class, "intake");
        this.isRed = isRed;
        telemetry.addData("isred", isRed);
        telemetry.update();
    }

    //intake action that is active as long as i press a button
    public Action intake(boolean buttonPressed) {
        this.buttonPressed = buttonPressed;
        power = INTAKE_SPEED;
        return new SetPowerAction();
    }
    //outtake action that is active as long as i press a button
    public Action outtake(boolean buttonPressed) {
        power = OUTTAKE_SPEED;
        return new SetPowerAction();
    }

    //intake action that intakes and outtake by right color
    public Action intakeByColor(boolean isSpecimen) {
        this.isSpecimen = isSpecimen;
        telemetry.addData("Color", colorSensorOur.isRightColor(true));
        telemetry.addData("Distance", colorSensorOur.getDistance());
        telemetry.addData("isSpecimen", isSpecimen);
        telemetry.addData("Speed", power);

        return new IntakeByColor();
    }


    public class SetPowerAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(power);
            return false;
        }
    }

    public class IntakeByColor implements Action {


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean colorCheek = colorSensorOur.isRightColor(isSpecimen);
            double dis = colorSensorOur.getDistance();
            boolean isIn = dis < 2;

            if (!isIn && !outtaking) {
                intake.setPower(INTAKE_SPEED);
            } else if (colorCheek) {
                intake.setPower(0);
            } else {
                intake.setPower(OUTTAKE_SPEED);
                if (!outtaking) {
                    timer.reset();
                    outtaking = true;

                } else {
                    outtaking = timer.milliseconds() < 750;
                }


            }

            return (!isIn && !colorCheek);
        }
    }
}