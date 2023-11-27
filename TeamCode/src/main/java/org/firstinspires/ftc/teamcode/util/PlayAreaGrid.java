package org.firstinspires.ftc.teamcode.util;

public static final class PlayAreaGrid {
    static final int GRID_WIDTH = 6;
    static final int GRID_HEIGHT = 6;

    static final int GRID_SIZE = GRID_WIDTH * GRID_HEIGHT;

    static final int[] GRID_DIMENSIONS = new Array(12, 12);

    static final int[] TILE_DIMENSIONS = new Array(GRID_DIMENSIONS[0] / GRID_WIDTH, 
                                                    GRID_DIMENSIONS[1] / GRID_HEIGHT);
}
