package org.firstinspires.ftc.teamcode.image;

public enum InitialPositions {
    POS1(200), POS2(400), POS3(600);

    private int maxXPos;

    InitialPositions(int maxXPos) {
        this.maxXPos = maxXPos;
    }

    public int getMaxXPos() {
        return maxXPos;
    }

    public boolean evalPos(int pos) {
        return pos < maxXPos;
    }
}
