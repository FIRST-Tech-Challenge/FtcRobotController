package org.firstinspires.ftc.teamcode.Util;

import android.graphics.Point;
import com.pedropathing.localization.Pose;

//Make your poses using THIS website: https://visualizer.pedropathing.com
//All tutorials and explanations can be found HERE: https://pedropathing.com

public class Poses {

    //Please Please PLEASE, use well constructed names for these poses.
    //The reason we use poses over points is that poses have a heading component.
    //Poses are for positions on the field, it literally stands for positional data.

    Pose startingPose = new Pose(72, 72, Math.toRadians(0));
    Pose endingPose = new Pose(36, 72, Math.toRadians(0));

    //The reason that we would use a POINT here is in the case that we need
    //to use a Bezier Curve. Bezier Curves are curved (citation needed) and
    //allow for a more efficient path than just a straight line in some cases.
    Point pathControlPoint = new Point(16, 16);







}
