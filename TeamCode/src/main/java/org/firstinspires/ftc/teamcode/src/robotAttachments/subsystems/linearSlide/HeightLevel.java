package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide;

import java.util.HashMap;

/**
 * A Enum for Height Levels
 */
public enum HeightLevel {
    Down,
    BottomLevel,
    MiddleLevel,
    TopLevel,
    GetOverObstacles;
    /**
     * Key is the Height Level, Value is the position to go to in ticks
     */
    protected static final HashMap<HeightLevel, Integer> EncoderCount = new HashMap<HeightLevel, Integer>() {{
        put(HeightLevel.BottomLevel, 0);
        put(HeightLevel.MiddleLevel, 233);
        put(HeightLevel.TopLevel, 584);
        put(HeightLevel.GetOverObstacles, 0);
        put(HeightLevel.Down, 0);
    }};

    public static Integer getEncoderCountFromEnum(HeightLevel level) {
        return EncoderCount.get(level);
    }
}
