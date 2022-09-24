package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class EndgameBot extends LEDBot{

    final double optimalDistance = 135000;

    double angleToGoal;
    double xTarget;
    double yTarget;

    public EndgameBot(LinearOpMode opMode) {
        super(opMode);
    }

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
    }



//    public void goToShoot(int count){
//        if (count == 1) {
//            angleToGoal = Math.toDegrees(Math.atan2((towerGoalX - xBlue)*-1, (towerGoalY - yBlue)*-1));
//            xTarget = (Math.sin(Math.toRadians(angleToGoal)) * optimalDistance) + towerGoalX;
//            yTarget = (Math.cos(Math.toRadians(angleToGoal)) * optimalDistance) + towerGoalY;
//        }
//        driveToCoordinate(xTarget, yTarget, angleToGoal+5, 4000, 0.8);
//        RobotLog.d(String.format("Angle: %.2f xTarget: %.2f yTarget: %.2f", angleToGoal, xTarget, yTarget));
//
//    }

}
