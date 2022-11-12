package org.firstinspires.ftc.teamcode.robots.catbot.util;

public class CranePositionMemory {
    private double MemoryTurretHeading;
    private double MemoryShoulderTick;
    private double MemoryExtendTick;

    public CranePositionMemory(double heading, double shoulder, double extend){
        MemoryTurretHeading = heading;
        MemoryShoulderTick = shoulder;
        MemoryExtendTick = extend;
    }

    public void setCranePositionMemory(double heading, double shoulder, double extend){
        MemoryTurretHeading = heading;
        MemoryShoulderTick = shoulder;
        MemoryExtendTick = extend;
    }

    public double getHeadingMemory(){
        return MemoryTurretHeading;
    }
    public double getShoulderMemory(){
        return MemoryShoulderTick;
    }
    public double getExtendMemory(){
        return MemoryExtendTick;
    }
}
