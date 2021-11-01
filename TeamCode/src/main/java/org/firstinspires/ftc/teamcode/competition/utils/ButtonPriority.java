package org.firstinspires.ftc.teamcode.competition.utils;

public class ButtonPriority {
    public boolean g1HasF1 = true;
    public boolean g1HasF2 = false;
    public boolean g1HasF3 = false;
    public boolean g1HasF4 = false;
    public boolean g1HasF5 = false;
    public boolean g1HasF6 = false;

    public boolean f1(boolean g1) { return g1 == g1HasF1; }
    public boolean f2(boolean g1) { return g1 == g1HasF2; }
    public boolean f3(boolean g1) { return g1 == g1HasF3; }
    public boolean f4(boolean g1) { return g1 == g1HasF4; }
    public boolean f5(boolean g1) { return g1 == g1HasF5; }
    public boolean f6(boolean g1) { return g1 == g1HasF6; }

    public void setG1HasF1(boolean g1HasF1) {
        this.g1HasF1 = g1HasF1;
    }

    public void setG1HasF2(boolean g1HasF2) {
        this.g1HasF2 = g1HasF2;
    }

    public void setG1HasF3(boolean g1HasF3) {
        this.g1HasF3 = g1HasF3;
    }

    public void setG1HasF4(boolean g1HasF4) {
        this.g1HasF4 = g1HasF4;
    }

    public void setG1HasF5(boolean g1HasF5) {
        this.g1HasF5 = g1HasF5;
    }

    public void setG1HasF6(boolean g1HasF6) {
        this.g1HasF6 = g1HasF6;
    }

}
