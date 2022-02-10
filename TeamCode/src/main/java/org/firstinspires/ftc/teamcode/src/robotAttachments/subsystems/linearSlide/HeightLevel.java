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

    private static final int[] heights;

    static {
        HeightLevel[] values = HeightLevel.values();
        heights = new int[values.length];
        for (int i = 0; i < values.length; i++) {
            heights[i] = HeightLevel.getEncoderCountFromEnum(values[i]);
        }
    }

    public static HeightLevel getClosestLevel(int currentLevel) {
        HeightLevel[] levels = HeightLevel.values();
        int[] distances = new int[levels.length];
        for (int i = 0; i < levels.length; i++) {
            distances[i] = Math.abs(heights[i] - currentLevel);
        }

        double smallestDistance = Double.MAX_VALUE;
        int indexOfSmallestValue = -1;

        for (int i = 0; i < levels.length; i++) {
            if (distances[i] < smallestDistance) {
                smallestDistance = distances[i];
                indexOfSmallestValue = i;
            }
        }
        return levels[indexOfSmallestValue];
    }

    private static int HeightLevelToInt(HeightLevel level) {
        switch (level) {

            case Down:
            case GetOverObstacles:
                return 0;
            case BottomLevel:
                return 1;
            case MiddleLevel:
                return 2;
            case TopLevel:
                return 3;
        }
        return 0;
    }

    private static HeightLevel intToHeightLevel(int value) {
        switch (value) {
            case 0:
                return Down;
            case 1:
                return BottomLevel;
            case 2:
                return MiddleLevel;
            case 3:
                return TopLevel;
        }

        if (value > 3) {
            return TopLevel;
        }
        return Down;

    }

    public HeightLevel add(int amount) {
        return HeightLevel.intToHeightLevel(HeightLevelToInt(this) + amount);
    }

    public HeightLevel subtract(int amount) {
        return HeightLevel.intToHeightLevel(HeightLevelToInt(this) - amount);
    }


}
