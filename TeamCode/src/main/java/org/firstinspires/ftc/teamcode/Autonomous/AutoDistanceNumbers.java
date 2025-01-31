package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoDistanceNumbers {

    //STEP 1
    public enum DriveForward {
        DRIVE_FORWARD_YAW (0),
        DRIVE_FORWARD_DISTANCE (19),
        DRIVE_FORWARD_POWER(-50);

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
        ARM_SLIGHT_DOWN(1450),
        UPARM_SLIGHT_BACK (-900);

        private final int value;
        HangSpecimen(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum DriveBehindSampleFromBar {

        IMU_TURN_POS (90),
        DISTANCE_AWAY_BAR (20),
        IMU_TURN_TO_SAMPLE (0),
        DISTANCE_TO_SAMPLE_1(20),
        DISTANCE_TO_SAMPLE_2(25),
        DISTANCE_TO_SAMPLE_3(30);

        private final int value;
        DriveBehindSampleFromBar(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum DriveBehindSampleFromObserve {

        IMU_TURN_TO_SAMPLE (90),
        DISTANCE_TO_SAMPLE_1(20),
        DISTANCE_TO_SAMPLE_2(25),
        DISTANCE_TO_SAMPLE_3(30);

        private final int value;
        DriveBehindSampleFromObserve(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum PushAndReverse {

        IMU_TURN_TO_SAMPLE (180),
        DISTANCE_TO_OBSERVATION(80),
        IMU_FACE_BACK(0);

        private final int value;
        PushAndReverse(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum GrabSpecimen {

        ARM_START_POS (2000),
        WAIT_TIME_MILLISECOND(500),
        IMU_FACE_BACK(0);

        private final int value;
        GrabSpecimen(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum DriveToInitialPos {

        IMU_FACE_INIT_POS (270),
        DISTANCE_FROM_PITSTOP_1(20),
        DISTANCE_FROM_PITSTOP_2(25),
        DISTANCE_FROM_PITSTOP_3(30);

        private final int value;
        DriveToInitialPos(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum DriveToObserve {
        IMU_FACE_INIT_POS (45),
        DISTANCE_TO_OBSERVE(30);

        private final int value;
        DriveToObserve(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum Park {
        IMU_FACE_PARK (225),
        DISTANCE_TO_OBSERVE(30);

        private final int value;
        Park(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }
}
