package org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.ServoAdvanced;

public class Intake {
    DcMotorEx motor;
    ServoAdvanced servo;
    HardwareMap hardwareMap;
    public static double LidOpen = 0;
    public static double LidClosed = 0.5;

    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        this.servo = new ServoAdvanced(hardwareMap.get(Servo.class, "intakeLid"));
    }

    public enum intakeState {
        INTAKE, STOP, OUTTAKE
    }

    public ElapsedTime timer = new ElapsedTime();

    public Action motorIntake(intakeState intakePos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double timeLastUpdate = timer.seconds();
                if (intakePos == intakeState.INTAKE) {
                    motor.setPower(0.75);
                    servo.setPosition(LidClosed);
                }
                if (intakePos == intakeState.OUTTAKE) {
                    motor.setPower(-0.75);
                    servo.setPosition(LidOpen);
                }
                if (intakePos == intakeState.STOP) {
                    motor.setPower(0);
                    servo.setPosition(LidOpen);
                }
                return false;
            }
        };
    }
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

