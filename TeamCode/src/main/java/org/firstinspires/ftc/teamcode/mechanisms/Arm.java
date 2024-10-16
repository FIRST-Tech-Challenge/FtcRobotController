package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.DcMotorExtensor;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Extensor;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;

public class Arm {
    public final Wrist wrist;
    public final Claw claw;
    public final Extensor extensor;

    public Arm(BaseRobot baseRobot) {
        claw = new Claw(baseRobot);
        wrist = new Wrist(baseRobot);
        extensor = new DcMotorExtensor(baseRobot);
    }
}