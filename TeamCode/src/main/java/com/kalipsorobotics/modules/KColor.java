package com.kalipsorobotics.modules;

public class KColor {

    public enum Color {
        RED, YELLOW, BLUE, NONE;
    }

    int red;
    int green;
    int blue;

    public int getGreen() {
        return green;
    }

    public void setGreen(int green) {
        this.green = green;
    }

    public int getRed() {
        return red;
    }

    public void setRed(int red) {
        this.red = red;
    }


    public int getBlue() {
        return blue;
    }

    public void setBlue(int blue) {
        this.blue = blue;
    }

    public KColor(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public KColor() {
    }
}
