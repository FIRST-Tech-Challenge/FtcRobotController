package org.firstinspires.ftc.teamcode.Mechanisms.Pivot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot.Robot;

@Config
public class Pivot {
    HardwareMap hardwareMap;
    Servo pivotLeft;
    Servo pivotRight;

    public static double pivotDown = 0;
    public static double pivotUp = 0.5;
    public Pivot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.pivotLeft = hardwareMap.get(Servo.class, "pivotLeft");
        this.pivotRight = hardwareMap.get(Servo.class, "pivotRight");

    }

    public ElapsedTime timer = new ElapsedTime();
    public Action flippyFlip(Intake.intakeState intakePos){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                    if (intakePos== Intake.intakeState.INTAKE) {
                        pivotLeft.setPosition(pivotDown);
                        pivotRight.setPosition(pivotDown);
                    }
                    else if (intakePos == Intake.intakeState.OUTTAKE) {
                        pivotLeft.setPosition(pivotDown);
                        pivotRight.setPosition(pivotDown);
                    } else if (intakePos == Intake.intakeState.STOP){
                        pivotLeft.setPosition(pivotUp);
                        pivotRight.setPosition(pivotUp);
                    }
                return false;
            }
        };
    }
}
