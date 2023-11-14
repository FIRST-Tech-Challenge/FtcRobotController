package org.firstinspires.ftc.teamcode;

public class RobotConfig {
    public int x;
    static RobotConfig instance;
    public static RobotConfig getInstance(){
        if (instance == null)
            instance = new RobotConfig();
        return instance;
    }
    public void init()
    {
        x = 0;
    }
}
