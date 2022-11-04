package org.firstinspires.ftc.teamcode.Reno;

public class RobotLocation {
    public double x;
    public double y;
    public double z;
    public double firstAngle;
    public double secondAngle;
    public double thirdAngle;
    public String targetName;

    public RobotLocation(double x, double y, double z, double firstAngle, double secondAngle, double thirdAngle)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        this.firstAngle = firstAngle;
        this.secondAngle = secondAngle;
        this.thirdAngle = thirdAngle;
    }

    public String toString()
    {
        return String.format("Robot location (" + this.targetName + ") (%5.2f, %5.2f), angle (%5.2f, %5.2f, %5.2f)", this.x, this.y, this.z, this.firstAngle, this.secondAngle, this.thirdAngle);
    }

    public void setTargetName(String targetName)
    {
        this.targetName = targetName;
    }

}
