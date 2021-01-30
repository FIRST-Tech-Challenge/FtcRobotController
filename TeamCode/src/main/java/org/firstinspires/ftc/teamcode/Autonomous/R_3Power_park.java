package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.OdometryChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "R_3Power_park")
public class R_3Power_park extends LinearOpMode {
    final boolean debug= true;
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        ElapsedTime runtime = new ElapsedTime();
        int rings  = robot.getRingsAndWaitForStart();
        robot.stopRingDetection();
       // waitForStart();
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
        robot.setPosition(-15.75,61.75, 0);//high goal -12,22
        if(rings==0) {
            robot.moveAngle( -5,-65, 0.8);//-25,60
            robot.moveAngle(41,7,0.8);//37,4
        }
        else if(rings==1) {
            robot.moveAngle(14,-89, 0.8);
            robot.moveAngle(10.5,31, 0.8);
        }
        else if(rings==4) {
            robot.moveAngle(-2.5, -113,1.0);
            robot.moveAngle(34,51.5,1.0);
        }
        robot.shootThreePowerShot();
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
        if(rings!=4) {

            if (rings == 0) {
                robot.turnInPlace(0, 1.0);
                robot.moveAngle(1, 49, 0.8);
                robot.moveAngle(-3,0,0.8);
                robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
                robot.moveAngle(-29,-59,0.7);
                robot.moveAngle(6,0,0.7);
            }
            if (rings == 1) {
                robot.turnInPlace(0, 1.0);
                robot.moveAngle(1, 48, 0.8);
                robot.turnInPlace(0,1.0);
                robot.moveAngle(-3,0,0.8);
                robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
                robot.moveAngle(-5,-80,0.7);
                robot.moveAngle(6,15,0.7);

            }
        }
        else{
            robot.turnInPlace(0, 1.0);
            robot.moveAngle(1, 49, 1.0);
            robot.turnInPlace(0,1.0);
            robot.moveAngle(-3,0,0.8);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.moveAngle(-27,-100,1.0);
            robot.moveAngle(15,40,1.0);
        }
        stop();
    }



}
