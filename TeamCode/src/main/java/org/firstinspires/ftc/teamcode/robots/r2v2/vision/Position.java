package org.firstinspires.ftc.teamcode.robots.r2v2.vision;

public enum Position {
    LEFT(0), MIDDLE(1), RIGHT(2), NONE_FOUND(-1), HOLD(-1);

    private final int index;

    Position(int index) {
        this.index = index;
    }

    public int getIndex() {
        return index;
    }
}
