package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorController;

public interface Encoder {
    class PositionVelocityPair {
        public final int position, velocity;

        public PositionVelocityPair(int position, int velocity) {
            this.position = position;
            this.velocity = velocity;
        }
    }

    PositionVelocityPair getPositionAndVelocity();

    DcMotorController getController();
}
