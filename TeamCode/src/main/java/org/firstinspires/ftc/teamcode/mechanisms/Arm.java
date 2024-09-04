package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Extender;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;

public class Arm {
    public final Wrist wrist;
    public final Claw claw;
    public final Extender extender;

    public Arm(BaseRobot baseRobot) {
        claw = new Claw(baseRobot);
        wrist = new Wrist(baseRobot);
        extender = new Extender(baseRobot);
    }
}