package org.firstinspires.ftc.teamcode.config;

public class AutoUtils {

    public static void tileStrafe(BaseOpMode baseOpMode, String direction, double numberOfTiles, double speed) {
        if (direction.equalsIgnoreCase("Left")) {
            DriveUtils.encoderStrafe(baseOpMode, speed, -numberOfTiles * 27, 10);
        } else {
            DriveUtils.encoderStrafe(baseOpMode, speed, numberOfTiles * 27, 10);
        }
    }

    public static void tileMove(BaseOpMode baseOpMode, String direction, double numberOfTiles, double speed) {
        if (direction.contains("F") || direction.contains("U")) {
            DriveUtils.encoderDrive(baseOpMode, speed, -numberOfTiles * 24, -numberOfTiles * 27, 10);
        } else {
            DriveUtils.encoderDrive(baseOpMode, speed, numberOfTiles * 24, numberOfTiles * 27, 10);
        }
    }

    // MAKE SURE TO MOVE THE CLAW UP 300 TICKS BEFORE YOU USE THIS METHOD
//    public enum ClawPositions {
//        ABOVE_GROUND(300),
//        MEDIUM_JUNCTION(1850),
//        DOWN_OVER_JUNCTION(-600),
//        FROM_OVER_JUNCTION_TO_STARTING_HEIGHT(-1550);
//        public final int encoderTicks;
//        ClawPositions(int encoderTicks) {
//            this.encoderTicks = encoderTicks;
//        }
//    }
    public static void moveClawToSpecificPosition(BaseOpMode baseOpMode, int numberOfTicks, double speed) {
        DriveUtils.encoderClaw(baseOpMode, 0.4, numberOfTicks, 5);


    }


}

