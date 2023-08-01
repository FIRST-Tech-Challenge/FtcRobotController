package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
public class RFMecanumDrive {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private RFPathFollower pathFollower;
    private RFPoseSim poseSim;
    public static boolean isPoseSim = true;
    public RFMecanumDrive(){
        pathFollower = new RFPathFollower();
        for (LynxModule module : op.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        if(!isPoseSim) {
            leftFront = op.hardwareMap.get(DcMotorEx.class, "motorLeftFront");
            leftRear = op.hardwareMap.get(DcMotorEx.class, "motorLeftBack");
            rightRear = op.hardwareMap.get(DcMotorEx.class, "motorRightBack");
            rightFront = op.hardwareMap.get(DcMotorEx.class, "motorRightFront");
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        poseSim = new RFPoseSim();
        currentVelocity = new Pose2d(0,0,0);
    }

    public void addWaypoint(RFWaypoint p_waypoint) {
        pathFollower.addWaypoint(p_waypoint);
    }
    public void setReversed(boolean p_reversed) {
        pathFollower.setReversed(p_reversed);
    }
    public void setDriveVelocity(Pose2d p_driveVelocity){
        double pF = p_driveVelocity.getX() * kV,
                pS = p_driveVelocity.getY() * kV,
                pR = 2 * TRACK_WIDTH * (p_driveVelocity.getHeading() * kV);
        double powers[] = {0,0,0,0};
        //frontLeft
        powers[0] = pF - pS - pR;
        //backLeft
        powers[1] = pF + pS - pR;
        //frontRight
        powers[2] = pF + pS + pR;
        //backRight
        powers[3] = pF - pS + pR;
        setMotorPowers(powers);
    }
    public void setJoystickPower(double p_y, double p_x, double p_a){
        Vector2d velocity = new Vector2d(p_y*MAX_VEL, p_x*MAX_VEL);
        double angle = p_a*MAX_ANG_VEL;
        double magnitude = velocity.norm()*velocity.norm()+angle*angle*4*TRACK_WIDTH*TRACK_WIDTH;
        if(magnitude>MAX_VEL*MAX_VEL){
            velocity.times(MAX_VEL/magnitude);
        }
        setDriveVelocity(new Pose2d(velocity,angle));
    }
    public void setTangentOffset(double p_tangentOffset) {
        pathFollower.setTangentOffset(p_tangentOffset);
    }
    public void setMotorPowers(double[] p_powers){
        if(!isPoseSim) {
            leftFront.setPower(p_powers[0]);
            leftRear.setPower(p_powers[1]);
            rightFront.setPower(p_powers[2]);
            rightRear.setPower(p_powers[3]);
        }
    }
    public void setPose(Pose2d p_pos){
        currentPose=p_pos;
    }
    public void update(){
        if(pathFollower.isFollowing()){
            if(isPoseSim){
                poseSim.updateSim();
                pathFollower.update();
                poseSim.getTargets(pathFollower.rfTrajectory.getTargetPosition(), pathFollower.PIDTargetVelocity);
            }
            else{
                setMotorPowers(pathFollower.update());
            }
        }
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
