package org.firstinspires.ftc.teamcode.robots.csbot.util;

public class CranePositionMemory {
    private double MemoryTurretHeading;
    private double MemoryDistance;
    private double MemoryHeight;

    public CranePositionMemory(double heading, double distance, double height){
        MemoryTurretHeading = heading;
        MemoryDistance = distance;
        MemoryHeight = height;
    }

    public void setCranePositionMemory(double heading, double distance, double height){
        MemoryTurretHeading = heading;
        MemoryDistance = distance;
        MemoryHeight = height;
    }

    public double getHeadingMemory(){
        return MemoryTurretHeading;
    }
    public double getDistanceMemory(){
        return MemoryDistance;
    }
    public double getHeightMemory(){
        return MemoryHeight;
    }
}
