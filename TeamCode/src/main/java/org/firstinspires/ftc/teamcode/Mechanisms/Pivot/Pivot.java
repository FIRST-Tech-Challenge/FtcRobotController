package org.firstinspires.ftc.teamcode.Mechanisms.Pivot;

import static org.firstinspires.ftc.teamcode.Mechanisms.Pivot.Pivot.pivotState;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;

@Config
public class Pivot {
    HardwareMap hardwareMap;
    Servo pivotLeft;
    Servo pivotRight;

    public static double pivotDown = 0;
    public static double pivotUp = 0.5;
    public pivotState pivotPos = pivotState.UP;
    public Pivot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.pivotLeft = hardwareMap.get(Servo.class, "pivotLeft");
        this.pivotRight = hardwareMap.get(Servo.class, "pivotRight");

    }

    public enum pivotState {
        UP, //spins to close servo, should only be closed enough to hold piece
        DOWN   //spins to have nearly fully open servo
    }
    public ElapsedTime timer = new ElapsedTime();
    public Action flippyFlip(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                double timeLastUpdate = timer.seconds();
                if (timeLastUpdate > 0.5) {
                    if (pivotPos == pivotState.DOWN) {
                        pivotLeft.setPosition(pivotUp);
                        pivotRight.setPosition(pivotUp);
                        pivotPos = pivotState.UP;
                    }
                    else if (pivotPos == pivotState.UP) {
                        pivotLeft.setPosition(pivotDown);
                        pivotRight.setPosition(pivotDown);
                        pivotPos = pivotState.DOWN;
                    }
                    timer.reset();
                }
                return false;
            }
        };
    }
}
