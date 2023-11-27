package org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

/*
    Utility for - you've guessed it! - storing a Twist2dDual<Time> object together with a timestamp.
*/

public class TwistWithTimestamp {
    public Twist2dDual<Time> twist;
    public double timestamp;

    public TwistWithTimestamp(Twist2dDual<Time> twist, double timestamp) {
        this.twist = twist;
        this.timestamp = timestamp;
    }
}
