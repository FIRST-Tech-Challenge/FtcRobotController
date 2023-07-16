package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.CVPipelines;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.VSLAMChassis.angle;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.VSLAMChassis.xpos;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.VSLAMChassis.ypos;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedTeamElem extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat= new Mat();
    public enum Location{
        LEFT,
        RIGHT,
        MID,
        NOT_FOUND
    }
    private RedTeamElem.Location location = RedTeamElem.Location.NOT_FOUND;

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
    static final Rect LEFT_ROI = new Rect( //130 x 210, 60 x 120
            new Point(185,240),
            new Point(115,150));
    static final Rect MIDDLE_ROI = new Rect( //310 x 210, 240 x 120
            new Point(320,240),
            new Point(250, 150));

    static double PERCENT_COLOR_THRESHOLD = 0.1; //percentage of color

    public RedTeamElem(){ telemetry=op.telemetry;}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV=new Scalar(75,50,50); // COLOR RANGE
        Scalar highHSV=new Scalar(125,255,255); //196,76,100

        Core.inRange(mat,lowHSV,highHSV,mat);
        Mat left = mat.submat(LEFT_ROI);
        Mat mid = mat.submat(MIDDLE_ROI);
        double leftValue= Core.sumElems(left).val[0]/LEFT_ROI.area()/255;
        double midValue= Core.sumElems(mid).val[0]/MIDDLE_ROI.area()/255;
        left.release();
        mid.release();

        telemetry.addData("Right raw Value",(int)Core.sumElems(left).val[0]);
        telemetry.addData("Right percentage ",Math.round(leftValue*100)+"%");

        telemetry.addData("Middle raw Value",(int)Core.sumElems(mid).val[0]);
        telemetry.addData("Middle percentage ",Math.round(midValue*100)+"%");

        boolean freightMid = midValue > PERCENT_COLOR_THRESHOLD;
        boolean freightLeft = leftValue > PERCENT_COLOR_THRESHOLD;

        if (freightLeft && true && freightMid){
            location = RedTeamElem.Location.NOT_FOUND;
            telemetry.addData("Object Location","not found");
        }
        else if (false){
            location = RedTeamElem.Location.RIGHT;
            telemetry.addData("Object Location","Left");
        }
        else if (freightLeft){
            location = RedTeamElem.Location.LEFT;
            telemetry.addData("Object Location","Right");
        }
        else if (freightMid){
            location = RedTeamElem.Location.MID;
            telemetry.addData("Object Location","Mid");
        }
        if(!freightMid&&!freightLeft){
            location = RedTeamElem.Location.NOT_FOUND;
        }
        telemetry.addData("x",xpos);
        telemetry.addData("y",ypos);
        telemetry.addData("a",angle);
        telemetry.update();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorRectangle= new Scalar(255,0,0);
        Imgproc.rectangle(mat,MIDDLE_ROI,location== RedTeamElem.Location.MID? colorRectangle:colorRectangle,5);
        Imgproc.rectangle(mat,LEFT_ROI,location== RedTeamElem.Location.LEFT? colorRectangle:colorRectangle,5);
        return mat;

    }
    public RedTeamElem.Location getLocation(){
        return location;
    }
}


//follow this tutorial "FTC EasyOpenCV Tutorial + SkyStone Example"

//https://www.youtube.com/watch?v=JO7dqzJi8lw&t=400s

