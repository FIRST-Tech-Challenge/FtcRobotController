package org.firstinspires.ftc.teamcode;


public class CommandRobotOpMode extends Robot {
    public enum OpModeType {
        TELEOP,
        AUTO
    }

    public CommandRobotOpMode(OpModeType type) {
        if (type == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    public void initTele() {

    }
}