package org.firstinspires.ftc.teamcode.Mechanisms.Intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;

public class Intake {
    DcMotorEx motor;
    HardwareMap hardwareMap;
    CRServo intakeServo;
    public Intake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
    }

    public enum intakeState {
        INTAKE, STOP, OUTTAKE
    }
    public ElapsedTime timer = new ElapsedTime();
    public Action motorIntake(intakeState intakePos){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                double timeLastUpdate = timer.seconds();
                    if(intakePos == intakeState.INTAKE){
                        motor.setPower(0.75);
                        intakeServo.setPower(-1);
                    }
                    if(intakePos == intakeState.OUTTAKE){
                        motor.setPower(-0.75);
                        intakeServo.setPower(1);
                    }
                    if (intakePos == intakeState.STOP){
                        motor.setPower(0);
                        intakeServo.setPower(0);
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
