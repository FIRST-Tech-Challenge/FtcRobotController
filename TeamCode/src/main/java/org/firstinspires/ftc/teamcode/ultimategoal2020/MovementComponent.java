package org.firstinspires.ftc.teamcode.ultimategoal2020;

import org.firstinspires.ftc.teamcode.ebotsenums.CoordinateSystem;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;

public class MovementComponent {
    private CoordinateSystem coordinateSystem;
    private CsysDirection csysDirection;
    private double distanceInches;

    public MovementComponent(CoordinateSystem csys, CsysDirection fd, double d){
        this.coordinateSystem = csys;
        this.csysDirection = fd;
        this.distanceInches = d;
    }

    public CoordinateSystem getCoordinateSystem() {
        return coordinateSystem;
    }

    public CsysDirection getCsysDirection() {
        return csysDirection;
    }

    public double getDistanceInches() {
        return distanceInches;
    }
}
