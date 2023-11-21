package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoOpModeConfig {
    public abstract static class AutoOpMode extends RobotOpMode {
        @Override
        public void gamePadMoveRobot() {}

        @Override
        public void init() {
            super.init();
        }
    }
}
