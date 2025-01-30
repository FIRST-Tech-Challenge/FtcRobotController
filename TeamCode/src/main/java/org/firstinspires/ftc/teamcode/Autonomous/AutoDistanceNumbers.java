package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoDistanceNumbers {

    //STEP 1
    public enum DriveForward {
        DRIVE_FORWARD_YAW (0),
        DRIVE_FORWARD_DISTANCE (15),
        DRIVE_FORWARD_POWER(-20);

        private final int value;
        DriveForward(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum HangSpecimen {
        ARM_HANG_POSITION (1265),
        UPARM_HANG_POSITION (-2550),
        DRIVE_FORWARD_POWER(-20);

        private final int value;
        HangSpecimen(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum DriveBehindSample {

        IMU_TURN_POS (90),
        DISTANCE_AWAY_BAR (20),
        IMU_TURN_TO_SAMPLE (0),
        DISTANCE_TO_SAMPLE(20);

        private final int value;
        DriveBehindSample(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

}
