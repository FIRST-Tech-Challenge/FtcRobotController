package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoDistanceNumbers {

    //STEP 1
    public enum DriveForward {
        DRIVE_FORWARD_YAW (0),
        DRIVE_FORWARD_DISTANCE (15),
        DRIVE_FORWARD_SPEED (-20);

        private final int value;
        DriveForward(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

}
