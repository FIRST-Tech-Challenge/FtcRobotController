package org.firstinspires.ftc.team6220_CENTERSTAGE;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
/*
This is part of roadrunner.
 */
public interface Localizer {
    Twist2dDual<Time> update();
}
