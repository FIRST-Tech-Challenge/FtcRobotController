package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class DriveAcceleration extends MainDrive {
    private double speedMultiplier = 1;
    private double increment = 0.01; // todo change this in the debug class

    public DriveAcceleration(OpMode opMode) {
        super(opMode);
    }

    public void checkInputs(
            float y,
            float x,
            float rx,
            boolean forwardButton,
            boolean reverseButton,
            boolean halfSpeedButton,
            boolean fourthSpeedButton
    ) {
        this.y = y;
        this.x = x;
        this.rx = rx;

        // this is literally all I added if it fixes acceleration i'm going to lose my mind
        if(this.y != 0 || this.x != 0) {
            if(speedMultiplier < 1) {
                speedMultiplier += increment;
            }
        } else {
            speedMultiplier = 0;

        }

        if (forwardButton) {
            direction = 1;
            setMotorDirections();
        } else if (reverseButton) {
            direction = -1;
            setMotorDirections();
        }

        // Slow speeds
        if (halfSpeedButton) {
            divideAmount = 2;
        } else if (fourthSpeedButton) {
            divideAmount = 4;
        } else {
            divideAmount = 1;
        }
        checkSpeed();

        updateMotors();
    }

    void checkSpeed() {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = ((y + x + (direction * rx)) / denominator) * speedMultiplier;
        backLeftPower = ((y - x + (direction * rx)) / denominator) * speedMultiplier;
        frontRightPower = ((y - x - (direction * rx)) / denominator) * speedMultiplier;
        backRightPower = ((y + x - (direction * rx)) / denominator) * speedMultiplier;
    }
}