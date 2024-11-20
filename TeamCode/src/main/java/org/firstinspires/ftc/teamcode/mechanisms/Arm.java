package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.GeckoIntake;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;
import org.firstinspires.ftc.teamcode.systems.Logger;

public class Arm {
    public final Wrist wrist;
    public final GeckoIntake intake;

    public Arm(BaseRobot baseRobot) {
        baseRobot.logger.add("Initializing Arm Components...", Logger.LogType.DEBUG);

        intake = new GeckoIntake(baseRobot);
        baseRobot.logger.add("✓ Claw initialized", Logger.LogType.DEBUG);

        wrist = new Wrist(baseRobot);
        baseRobot.logger.add("✓ Wrist initialized", Logger.LogType.DEBUG);

        baseRobot.logger.add("All Arm Components Initialized", Logger.LogType.DEBUG);
    }
}