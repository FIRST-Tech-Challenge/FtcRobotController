package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueTeamElem extends OpenCvPipeline {
    LinearOpMode op;
    Telemetry telemetry;
    Mat mat= new Mat();
    public enum Location{
        LEFT,
        RIGHT,
        MID,
        NOT_FOUND
    }
    private Location location = Location.NOT_FOUND;

/* Nathan's values
    static final Rect LEFT_ROI = new Rect(
            new Point(-50, 50), //
            new Point( 50,150));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(200,350),
            new Point(300 ,400));
    static final Rect RIGHT_ROI = new Rect(
            new Point(500,  575),
            new Point(600,  675));*/

    //New calculations
    static final Rect MIDDLE_ROI = new Rect(
            new Point(80,130),
            new Point(20,40));
    static final Rect RIGHT_ROI = new Rect(
            new Point(230, 130),
            new Point(170,40));

    static double PERCENT_COLOR_THRESHOLD = 0.3; //percentage of color

    public BlueTeamElem(LinearOpMode opMode){op=opMode; telemetry=op.telemetry;}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV=new Scalar(75,50,50); // COLOR RANGE
        Scalar highHSV=new Scalar(125,255,255); //196,76,100

        Core.inRange(mat,lowHSV,highHSV,mat);
        Mat right = mat.submat(RIGHT_ROI);
        Mat mid = mat.submat(MIDDLE_ROI);
        double rightValue= Core.sumElems(right).val[0]/RIGHT_ROI.area()/255;
        double midValue= Core.sumElems(mid).val[0]/MIDDLE_ROI.area()/255;
        right.release();
        mid.release();

        telemetry.addData("Right raw Value",(int)Core.sumElems(right).val[0]);
        telemetry.addData("Right percentage ",Math.round(rightValue*100)+"%");

        telemetry.addData("Middle raw Value",(int)Core.sumElems(mid).val[0]);
        telemetry.addData("Middle percentage ",Math.round(midValue*100)+"%");

        boolean freightMid = midValue > PERCENT_COLOR_THRESHOLD;
        boolean freightRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (freightRight && true && freightMid){
            location = Location.NOT_FOUND;
            telemetry.addData("Object Location","not found");
        }
        else if (false){
            location = Location.LEFT;
            telemetry.addData("Object Location","Left");
        }
        else if (freightRight){
            location = Location.RIGHT;
            telemetry.addData("Object Location","Right");
        }
        else if (freightMid){
            location = Location.MID;
            telemetry.addData("Object Location","Mid");
        }
        telemetry.update();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorRectangle= new Scalar(255,0,0);
        Imgproc.rectangle(mat,MIDDLE_ROI,location== Location.MID? colorRectangle:colorRectangle,5);
        Imgproc.rectangle(mat,RIGHT_ROI,location== Location.RIGHT? colorRectangle:colorRectangle,5);
        return mat;

}
    public Location getLocation(){
        return location;
    }
}


//follow this tutorial "FTC EasyOpenCV Tutorial + SkyStone Example"

//https://www.youtube.com/watch?v=JO7dqzJi8lw&t=400s

