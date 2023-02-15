package org.firstinspires.ftc.teamcode.teamUtil;

public class Pose2D {
    public Coordinate2D coordinate2D;
    public Angle angle;

    public Pose2D(Coordinate2D coordinate2D, Angle angle){
        this.coordinate2D = coordinate2D;
        this.angle = angle;
    }

    public Pose2D(){
        this.coordinate2D = new Coordinate2D(0, 0);
        this.angle = new Angle(90);
    }

    public static Pose2D moduleProcessor(Pose2D leftModulePose2D, Pose2D rightModulePose2D, Angle heading, RobotConstants.enabledModules enabledModules){ //TODO change this to use the new imu and/or odometery in the new sdk version 8.1, rather than using module tracking
        Pose2D robotPose2D;
        switch (enabledModules){
            case LEFT:
                robotPose2D = new Pose2D(leftModulePose2D.coordinate2D, new Angle(90, Angle.angleType.ABSOLUTE));
                break;

            case RIGHT:
                robotPose2D = new Pose2D(rightModulePose2D.coordinate2D, new Angle(90, Angle.angleType.ABSOLUTE));
                break;

            case BOTH:
                //outdated thingo, it uses the module positions to find the heading robotPose2D = new pose2D(new coordinate2D(((leftModulePose2D.coordinate2D.x+rightModulePose2D.coordinate2D.x)/2), ((leftModulePose2D.coordinate2D.y+rightModulePose2D.coordinate2D.y)/2)), new angle(org.firstinspires.ftc.teamcode.teamUtil.angle.atanHandler(leftModulePose2D.coordinate2D.x-rightModulePose2D.coordinate2D.x, leftModulePose2D.coordinate2D.y-rightModulePose2D.coordinate2D.y).value+90, org.firstinspires.ftc.teamcode.teamUtil.angle.angleType.ABSOLUTE));
                robotPose2D = new Pose2D(new Coordinate2D(((leftModulePose2D.coordinate2D.x+rightModulePose2D.coordinate2D.x)/2), ((leftModulePose2D.coordinate2D.y+rightModulePose2D.coordinate2D.y)/2)), heading);

                break;

            default:
                robotPose2D = new Pose2D(new Coordinate2D(0,0), new Angle(0, Angle.angleType.ABSOLUTE));
                break;
        }
        return robotPose2D;
    }

    public double getDifference(Pose2D other){
        return Math.sqrt((other.coordinate2D.x-this.coordinate2D.x)*(other.coordinate2D.x-this.coordinate2D.x)+(other.coordinate2D.y-this.coordinate2D.y)*(other.coordinate2D.y-this.coordinate2D.y));
    }

}
