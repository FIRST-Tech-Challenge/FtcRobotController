package org.firstinspires.ftc.teamcode.GoalDetector;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;

public class Contour
{
    public MatOfPoint self;
    public Contour parent;
    public double area;
    public ArrayList<Contour> children = new ArrayList<>();
    public int originalListIdx = 0;
}
