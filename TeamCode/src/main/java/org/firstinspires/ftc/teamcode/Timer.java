package org.firstinspires.ftc.teamcode;

public class Timer {
    Timer(String name) {
        this.name = name;
    }
    private String name;
    private int time = 0;

    void reset() { time = 0; }
    void updateTime() { time++; }
    int getElapsedTime() { return time; }
    String getName() { return name; }
}
