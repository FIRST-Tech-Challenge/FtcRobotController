package org.firstinspires.ftc.teamcode.util;

import org.opencv.imgproc.Moments;

/**
 * Created by IronReign on 3/28/2018.
 */

public class BlobStats {


    public Moments moments;
    public int x;
    public int y;
    public int width;
    public int height;
    public double area;


public BlobStats(Moments moments, int x, int y, int width, int height, double area){
    this.moments = moments;
    this.x = x;
    this.y = y;
    this.width = width;
    this.height = height;
    this.area = area;
}
}