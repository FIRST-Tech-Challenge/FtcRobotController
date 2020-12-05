package org.firstinspires.ftc.teamcode.lib.geometry;

public interface ICurvature<S> extends State<S> {
    double getCurvature();

    double getDCurvatureDs();
}
