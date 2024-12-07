package org.firstinspires.ftc.teamcode.Mechanisms.Intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;

public class Intake {
    DcMotorEx motor;
    HardwareMap hardwareMap;
    public IntakeState intakePos = IntakeState.STOP;
    public Intake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
    }

    public enum IntakeState {
        INTAKE, STOP, OUTTAKE
    }
    public ElapsedTime timer = new ElapsedTime();
    public Action motorIntake(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                double timeLastUpdate = timer.seconds();
                if (timeLastUpdate > 0.5) {
                    if(intakePos == IntakeState.INTAKE){
                        motor.setPower(0.75);
                    }
                    if(intakePos == IntakeState.OUTTAKE){
                        motor.setPower(-0.75);
                    }
                    timer.reset();
                }
                return false;
            }
        };
    }


//    public void intakeIn(){
//        motor.setPower(0.5);
//    }
//    public void intakeOut(){
//        motor.setPower(-0.5);
//    }
//
//    public Action outtake(){
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                motor.setPower(-0.75);
//                return gamepad1.left_bumper;
//            }
//        };
//    }
}
