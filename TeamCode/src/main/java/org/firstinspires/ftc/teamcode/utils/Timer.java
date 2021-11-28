package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Dictionary;
import java.util.Enumeration;

public class Timer{

    private Dictionary<String, ElapsedTime> timers;

    public Timer () {}

    public void addTimer(String name) {
        timers.put(name, new ElapsedTime());
    }

    public ElapsedTime getTimer(String name) {
        return timers.get(name);
    }

    public Enumeration<String> getAllTimers() {
        return timers.keys();
    }
}
