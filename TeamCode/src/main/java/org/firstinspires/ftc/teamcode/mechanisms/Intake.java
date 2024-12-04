package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.GeckoWheels;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.HorizontalSlide;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;
import org.firstinspires.ftc.teamcode.systems.Logger;

public class Intake {
    public final Wrist wrist;
    public final GeckoWheels geckoWheels;

    public Intake(BaseRobot baseRobot) {
        geckoWheels = new GeckoWheels(baseRobot);
        wrist = new Wrist(baseRobot);
    }
}