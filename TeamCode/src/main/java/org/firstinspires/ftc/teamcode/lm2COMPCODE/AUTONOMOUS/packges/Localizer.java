package org.firstinspires.ftc.teamcode.lm2COMPCODE.AUTONOMOUS.packges;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

public interface Localizer {
    Twist2dDual<Time> update();
}
