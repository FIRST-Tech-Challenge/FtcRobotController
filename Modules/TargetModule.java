package org.firstinspires.ftc.teamcode.rework.Modules;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Skystone.Accessories.MathFunctions;
import org.firstinspires.ftc.teamcode.rework.ModuleTools.Module;
import org.firstinspires.ftc.teamcode.rework.Robot;

import static org.firstinspires.ftc.teamcode.rework.MathFunctions.*;

public class TargetModule implements Module {

    Robot robot;

    public boolean isTargeting = false;
    public double targetPointX;
    public double targetPointY;

    public double moveSpeed = 1;
    public double turnSpeed = 1;

    public TargetModule(Robot robot) {
        this.robot = robot;
    }

    public void init() {

    }

    public void update() {
        double distanceToTarget = Math.hypot(targetPointX - robot.odometryModule.worldX, targetPointY - robot.odometryModule.worldY);
        double absoluteAngleToTarget = Math.atan2(targetPointX - robot.odometryModule.worldX, targetPointY - robot.odometryModule.worldY);

        double relativeAngleToPoint = absoluteAngleToTarget - robot.odometryModule.worldAngleRad;
        double relativeXToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeTurnAngle = angleWrap2(relativeAngleToPoint);

        // adjust vector based on current velocity
        relativeXToPoint -= 0.1 * robot.velocityModule.xVel;
        relativeYToPoint -= 0.1 * robot.velocityModule.yVel;

        double xPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

        robot.drivetrainModule.xMovement = xPower * moveSpeed;
        robot.drivetrainModule.yMovement = yPower * moveSpeed;
        robot.drivetrainModule.turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

//        if (distanceToTarget > 0.5){
//
//        } else {
//            robot.drivetrainModule.xMovement = 0;
//            robot.drivetrainModule.yMovement = 0;
//            robot.drivetrainModule.turnMovement = 0;
//        }
    }
}
