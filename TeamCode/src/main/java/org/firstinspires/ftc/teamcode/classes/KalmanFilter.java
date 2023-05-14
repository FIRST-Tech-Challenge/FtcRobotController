package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.util.ElapsedTime;

public class KalmanFilter {

    private double currentState;
    private double pastState;
    private ElapsedTime timer;

    public KalmanFilter(double currentState) {
        this.currentState = currentState;
    }
}
