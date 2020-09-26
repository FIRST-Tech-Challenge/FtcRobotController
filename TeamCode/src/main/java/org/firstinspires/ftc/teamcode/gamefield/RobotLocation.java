package org.firstinspires.ftc.teamcode.gamefield;

public class RobotLocation {
    public float X = 0f;
    public float Y = 0f;
    public float Z = 0f;

    public float rotateX = 0f;
    public float rotateY = 0f;
    public float rotateZ = 0f;

    private boolean active =false;


    public RobotLocation(boolean active){
        this.setActive(active);
    }

    public boolean isActive() {
        return active;
    }

    public void setActive(boolean active) {
        this.active = active;
    }
}
