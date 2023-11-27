package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;

public final class PlayAreaGrid {
    static final int GRID_WIDTH = 6;
    static final int GRID_HEIGHT = 6;

    static final int GRID_SIZE = GRID_WIDTH * GRID_HEIGHT;

    static final int[] GRID_DIMENSIONS = {12, 12};

    static final int[] TILE_DIMENSIONS = {GRID_DIMENSIONS[0] / GRID_WIDTH,
                                                    GRID_DIMENSIONS[1] / GRID_HEIGHT};
}
