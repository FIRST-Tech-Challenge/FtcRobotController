package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Accelerator {
    ElapsedTime accelerationTimer;
    private boolean isAccelerateCycle = false;

    public Accelerator(){
        accelerationTimer = new ElapsedTime();
    }

    public double update(double power){
        if (power == 0) {
            isAccelerateCycle = false;
            return 0.0;
        }

        if (!isAccelerateCycle){
            accelerationTimer.reset();
            isAccelerateCycle = true;
        }

        double accelerationFactor = (Math.tanh(0.5 * accelerationTimer.seconds() - 1.5) / 2.5) + 0.6;
        power *= accelerationFactor;

        if (power > 1) power = 1;
        else if (power < -1) power = -1;

        return power;
    }
}
