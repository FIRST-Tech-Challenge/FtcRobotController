package org.firstinspires.ftc.teamcode.teamUtil;

public class pose2D {
    public coordinate2D coordinate2D;
    public angle angle;

    public pose2D(coordinate2D coordinate2D, angle angle){
        this.coordinate2D = coordinate2D;
        this.angle = angle;
    }

    public static pose2D moduleProcessor(pose2D leftModulePose2D, pose2D rightModulePose2D, robotConstants.enabledModules enabledModules){ //TODO change this to use the new imu and/or odometery in the new sdk version 8.1, rather than using module tracking
        pose2D robotPose2D;
        switch (enabledModules){
            case LEFT:
                robotPose2D = new pose2D(leftModulePose2D.coordinate2D, new angle(90, org.firstinspires.ftc.teamcode.teamUtil.angle.angleType.ABSOLUTE));
                break;

            case RIGHT:
                robotPose2D = new pose2D(rightModulePose2D.coordinate2D, new angle(90, org.firstinspires.ftc.teamcode.teamUtil.angle.angleType.ABSOLUTE));
                break;

            case BOTH:
                robotPose2D = new pose2D(new coordinate2D(((leftModulePose2D.coordinate2D.x+rightModulePose2D.coordinate2D.x)/2), ((leftModulePose2D.coordinate2D.y+rightModulePose2D.coordinate2D.y)/2)), new angle(org.firstinspires.ftc.teamcode.teamUtil.angle.atanHandler(leftModulePose2D.coordinate2D.x-rightModulePose2D.coordinate2D.x, leftModulePose2D.coordinate2D.y-rightModulePose2D.coordinate2D.y).value+90, org.firstinspires.ftc.teamcode.teamUtil.angle.angleType.ABSOLUTE));
                break;

            default:
                robotPose2D = new pose2D(new coordinate2D(0,0), new angle(0, org.firstinspires.ftc.teamcode.teamUtil.angle.angleType.ABSOLUTE));
                break;
        }
        return robotPose2D;
    }

    public double getDifference(pose2D other){
        return Math.sqrt((other.coordinate2D.x-this.coordinate2D.x)*(other.coordinate2D.x-this.coordinate2D.x)+(other.coordinate2D.y-this.coordinate2D.y)*(other.coordinate2D.y-this.coordinate2D.y));
    }

}
