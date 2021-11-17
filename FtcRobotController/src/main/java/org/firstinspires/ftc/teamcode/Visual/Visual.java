package org.firstinspires.ftc.teamcode.Visual;

import org.firstinspires.ftc.teamcode.common.Assembly;
import org.firstinspires.ftc.teamcode.common.Position;
import java.io.PrintWriter;


public abstract class Visual extends Assembly {

    public enum findTeamElement {
        Left,
        Center,
        Right,
    }

    public abstract Position getPosition();

    public abstract findTeamElement findTeamElement();

    public abstract double getTeamElementOffset();

    public abstract boolean[] isBlack(PrintWriter logging);

    public abstract void stop();
}