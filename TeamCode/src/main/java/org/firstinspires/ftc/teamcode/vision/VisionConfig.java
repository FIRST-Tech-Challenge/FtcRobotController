package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.opencv.ColorRange;

/** Shared settings for the team's vision experiments. */
public final class VisionConfig {
    // These names must match the Robot Controller hardware configuration.
    public static final String WEBCAM_NAME = "Webcam 1";
    public static final String HUSKYLENS_NAME = "huskylens";

    // Camera coordinates: (0, 0) is the upper-left corner of the image.
    public static final int FRAME_WIDTH_PX = 640;
    public static final int FRAME_HEIGHT_PX = 480;

    // Starter example: identify yellow objects by color, not by a trained AI model.
    // Calibrate this range and size threshold using the actual competition pieces.
    public static final ColorRange TARGET_COLOR = ColorRange.YELLOW;
    public static final String TARGET_LABEL = "yellow_game_piece";
    public static final int MIN_BLOB_AREA_PX = 300;

    private VisionConfig() {
        // Utility class: no instances.
    }
}
