package org.firstinspires.ftc.teamcode.Mechanisms.Sweeper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.ServoAdvanced;

@Config
public class Sweeper {
    HardwareMap hardwareMap;
    ServoAdvanced sweeper;
    public static double sweeperBackwards = 0;
    public static double sweeperForwards = 1;
    sweepState status = sweepState.BACKWARD;
    private boolean runSweeper = false;
    public void startSweeper(){
        runSweeper = true;
    }
    public void stopSweeper(){
        runSweeper = false;
    }
    public ElapsedTime timer = new ElapsedTime();
    public Sweeper(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.sweeper = new ServoAdvanced(hardwareMap.get(Servo.class, "sweeper"));
    }
    public enum sweepState {
        FORWARD,    // pulls arm in
        BACKWARD
    }
    ElapsedTime sweeperTimer = new ElapsedTime();
    public Action servoArmSpec(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                if (sweeperTimer.seconds()>.3){
                    if (status==sweepState.FORWARD) {
                        sweeper.setPosition(sweeperBackwards);
                        status=sweepState.BACKWARD;
                    }
                    else if (status==sweepState.BACKWARD) {
                        sweeper.setPosition(sweeperForwards);
                        status=sweepState.BACKWARD;
                    }
                }
                return false;
            }
        };
    }
}