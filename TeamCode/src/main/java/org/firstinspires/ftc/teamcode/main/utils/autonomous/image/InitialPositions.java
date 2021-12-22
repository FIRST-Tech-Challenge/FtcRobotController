package org.firstinspires.ftc.teamcode.main.utils.autonomous.image;

public enum InitialPositions {
    POS1(200, "1"), POS2(400, "2"), POS3(600, "3");

    private int maxXPos;
    private String name;

    InitialPositions(int maxXPos, String name) {
        this.maxXPos = maxXPos;
        this.name = name;
    }

    public int getMaxXPos() {
        return maxXPos;
    }

    public boolean evalPos(int pos) {
        return pos < maxXPos;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }
}
