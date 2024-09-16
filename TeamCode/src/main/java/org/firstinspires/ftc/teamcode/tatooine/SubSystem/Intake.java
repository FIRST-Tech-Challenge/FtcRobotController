package org.firstinspires.ftc.teamcode.tatooine.SubSystem;


import android.drm.DrmStore;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake  {
    CRServo intake = null;
    private double power;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(CRServo.class, "intake");
    }
    //no action
    public void setPower(double power) {
        intake.setPower(power);
    }
    //uses action and i chose to not give it a constent power(a power that the user give)
    public Action setPowerAction(double power){
        this.power = power;
        return new SetPowerAction();
    }

    public class SetPowerAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(power);
            return power>0;
        }
    }
}