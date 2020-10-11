package org.firstinspires.ftc.teamcode.statemachine;

public class Stage {

    private int stage;

    public Stage() {
        this(0);
    }

    private Stage(int stage) {
        this.stage = stage;
    }

    public int getStage() {
        return stage;
    }

    public void incrementStage() {
        stage++;
    }

    public void resetStage() {
        stage = 0;
    }

    @Override
    public String toString() {
        return Integer.toString(stage);
    }

}
