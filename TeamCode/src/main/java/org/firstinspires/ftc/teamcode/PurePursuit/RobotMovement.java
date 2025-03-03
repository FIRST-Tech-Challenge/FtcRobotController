package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.atTarget;
import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.calculateAngleUnwrap;
import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.calculateCircleIntersection;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Vector;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.WayPoint;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotMap;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.TwoDeadWheelLocalizer;

import java.util.ArrayList;

public class RobotMovement {

    private RobotMap robotMap;
    private TwoDeadWheelLocalizer localizer;
    private ElapsedTime time;

    public volatile boolean isFinished, nullDetected;
    public volatile Vector currentPoint;

    public volatile Pose pose = new Pose(0, 0, 0);
    private Pose2d tempPose = new Pose2d(0,0,0);

    public double
        previousTime,
        positionTargetVelocity,
        rotationalTargetVelocity,
        currentRadius,
        currentMaxVelocity,
        currentMaxAcceleration,
        currentMaxRotationalVelocity,
        currentMaxRotationalAcceleration,
        currentMinRadiusRange,
        currentMaxRadiusRange;

    public RobotMovement(HardwareMap hm, Telemetry telemetry) {

        robotMap = new RobotMap(hm, telemetry);
        localizer = new TwoDeadWheelLocalizer(hm, robotMap);
        time = new ElapsedTime();
    }

    public void followPathUpdate(ArrayList <WayPoint> points) {

        isFinished = false;

        Pose motorsPower;

        WayPoint followPoint = points.get(0);

        WayPoint start =
            new WayPoint.WaypointBuilder(new Pose(0,0,0),
                                         WayPoint.WaypointType.DEFAULT).build();

        WayPoint end =
            new WayPoint.WaypointBuilder(new Pose(0,0,0),
                                         WayPoint.WaypointType.DEFAULT).build();

        while(!isFinished) {
            time.reset();

            tempPose = tempPose.plus(localizer.update().value());
            pose.pose2dTranslation(tempPose);

            for (int i = points.size() - 2; i >= 0; --i) {
                start = points.get(0);
                end = points.get(0 + 1);

                currentMaxVelocity = end.max_vel;
                currentMaxAcceleration = end.max_accel;
                currentMaxRotationalVelocity = end.max_rot_vel;
                currentMaxRotationalAcceleration = end.max_rot_accel;
                currentMinRadiusRange = end.min_radius;
                currentMaxRadiusRange = end.max_radius;

//                currentRadius = calculateRadius(getCurrentVelocity());

                if (atTarget(pose, end)) isFinished = true;

                currentPoint = calculateCircleIntersection(
                    new Vector(pose.x, pose.y),
                    currentRadius,
                    new Vector(start.pose.x, start.pose.y),
                    new Vector(end.pose.x, end.pose.y)
                );

                if (currentPoint != null){
                    followPoint.setPoint(currentPoint);
                    break;
                } else {
                    nullDetected = true;
                    currentRadius += currentRadius/10;
                }
            }

//            positionMotionProfiling(end);
//            rotationMotionProfiling(end);

            motorsPower = goToPoint(followPoint.pose);

            robotCentricMovement(motorsPower);
    }

    public Pose goToPoint(Pose followPose) {
        Pose answer = new Pose(0,0,0);

        double distanceToTarget = Math.hypot(followPose.x - pose.x, followPose.y - pose.y);
        double targetAngle = Math.toDegrees(
            Math.atan2(followPose.y - pose.y, followPose.x - pose.x));
        double angleError = calculateAngleUnwrap(targetAngle - pose.theta);

        double relativeXToPoint = Math.cos(angleError) * distanceToTarget;
        double relativeYToPoint = Math.sin(angleError) * distanceToTarget;

        double movementXPower = relativeXToPoint / distanceToTarget;
        double movementYPower = relativeYToPoint / distanceToTarget;

        double relativeTurnAngle = angleError - 180 + followPose.theta;
        // TODO: make sure relativeTurnAngle is between -180 and 180
        // Not sure the - 180 is correct must check

        answer.x = movementXPower * positionTargetVelocity;
        answer.y = movementYPower * positionTargetVelocity;
        answer.theta = relativeTurnAngle * rotationalTargetVelocity;

        return answer;
    }

    public void robotCentricMovement(Pose pose) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(pose.y) + abs(pose.x) + abs(pose.theta), 1);
        double frontLeftPower = (pose.y + pose.x + pose.theta) / denominator;
        double backLeftPower = (pose.y - pose.x + pose.theta) / denominator;
        double frontRightPower = (pose.y - pose.x - pose.theta) / denominator;
        double backRightPower = (pose.y + pose.x - pose.theta) / denominator;

        robotMap.getFrontLeft().setPower(frontLeftPower);
        robotMap.getRearLeftMotor().setPower(backLeftPower);
        robotMap.getFrontRight().setPower(frontRightPower);
        robotMap.getRearRightMotor().setPower(backRightPower);
    }

    public double getCurrentVelocity() {
        return localizer.getCurrentVelocity();
    }


    public double getCurrentRotationalVelocity() {
        return localizer.getCurrentRotationalVelocity();
    }
}