package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide;


/**
 * A Enum for Height Levels
 */
public enum HeightLevel {
    Down,
    BottomLevel,
    MiddleLevel,
    TopLevel,
    CappingUp,
    CappingDown,
    GetOverObstacles;

    /**
     * Pass in the Height Level, returns the the position to go to in ticks
     */
    private static int getEncoderCountFromLevel(HeightLevel level) {
        switch (level) {
            case TopLevel:
                return 584;
            case MiddleLevel:
                return 233;

            case CappingUp:
                return 500;

            case CappingDown:
                return 335;
            case BottomLevel:
            case Down:
            case GetOverObstacles:
            default:
                return 0;
        }
    }

    public static int getEncoderCountFromEnum(HeightLevel level) {
        return getEncoderCountFromLevel(level);
    }
}
