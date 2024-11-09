package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.DcMotorExtensor;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Extensor;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Shoulder;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.SingleMotorExtensorJustForConnerBecauseHeWantsToRebuildTheRobotOnAWhim;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;

public class Arm {
    public final Wrist wrist;
    public final Claw claw;
    public final Extensor extensor;
    public final Shoulder shoulder;

    public Arm(BaseRobot baseRobot) {
        baseRobot.logger.add("Initializing Arm Components...", Logger.LogType.DEBUG);

        claw = new Claw(baseRobot);
        baseRobot.logger.add("✓ Claw initialized", Logger.LogType.DEBUG);

        wrist = new Wrist(baseRobot);
        baseRobot.logger.add("✓ Wrist initialized", Logger.LogType.DEBUG);

        extensor = new SingleMotorExtensorJustForConnerBecauseHeWantsToRebuildTheRobotOnAWhim(baseRobot);
        baseRobot.logger.add("✓ Extensor initialized", Logger.LogType.DEBUG);

        shoulder = new Shoulder(baseRobot);
        baseRobot.logger.add("✓ Shoulder initialized", Logger.LogType.DEBUG);

        baseRobot.logger.add("All Arm Components Initialized", Logger.LogType.DEBUG);
    }
}