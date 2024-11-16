package org.firstinspires.ftc.teamcode.Mechanisms.Intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    DcMotorEx motor;
    HardwareMap hardwareMap;

    public Intake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
    }
    public void intakeIn(){
        motor.setPower(0.5);
    }
    public void intakeOut(){
        motor.setPower(-0.5);
    }


    public Action intake(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                motor.setPower(0.5);
                return gamepad1.right_bumper;
            }
        };
    }

    public Action outtake(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                motor.setPower(-0.5);
                return gamepad1.left_bumper;
            }
        };
    }
}
