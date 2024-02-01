package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Deprecated
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
