package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RFMecanumDrive {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private RFPathFollower pathFollower;
    public RFMecanumDrive(){
        pathFollower = new RFPathFollower();
        for (LynxModule module : op.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        leftFront = op.hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        leftRear = op.hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        rightRear = op.hardwareMap.get(DcMotorEx.class, "motorRightBack");
        rightFront = op.hardwareMap.get(DcMotorEx.class, "motorRightFront");
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void addWaypoint(RFWaypoint p_waypoint) {
        pathFollower.addWaypoint(p_waypoint);
    }
    public void setReversed(boolean reversed) {
        pathFollower.setReversed(reversed);
    }

    public void setTangentOffset(double p_tangentOffset) {
        pathFollower.setTangentOffset(p_tangentOffset);
    }

    public void setConstantHeading(boolean p_constantHeading) {
        pathFollower.setConstantHeading(p_constantHeading);
    }

    public void setCurviness(double p_curviness) {
        pathFollower.setCurviness(p_curviness);
    }
    public void changeEndpoint(RFWaypoint p_endpoint) {
        pathFollower.changeEndpoint(p_endpoint);
    }

    public void eraseWaypoints() {
        pathFollower.eraseWaypoints();
    }

    public boolean isFollowing() {
        return pathFollower.isFollowing();
    }
}
