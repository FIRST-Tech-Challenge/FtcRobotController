package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "R_3Power_park")
public class R_3Power_park extends LinearOpMode {
    final boolean debug= true;
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        ElapsedTime runtime = new ElapsedTime();

        int rings = robot.getRingsAndWaitForStart();

//        waitForStart();
//        rings = robot.tensorFlow.getNumberOfRings();
        robot.stopRingDetection();
        telemetry.update();
        //waitForStart();
        if(rings==0) {
            robot.moveAngle( -25,-65, 0.6);
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(0,2,0.8);
            robot.turnInPlace(0,1.0);
            robot.moveAngle(37,2,0.5);
            robot.turnInPlace(2.0,1.0);
        }
        else if(rings==1) {
            robot.moveAngle(4,-83, 0.7);
            robot.turnInPlace(0,0.5);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(0,2.5, 0.7);
            robot.turnInPlace(0,0.5);
            robot.moveAngle(11,22, 0.7);
            robot.turnInPlace(0,1.0);
        }
        else if(rings==4) {
            robot.moveAngle(-25, -110,0.7);
            robot.turnInPlace(0,0.5);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(-0, 7,0.7);
            robot.turnInPlace(0,0.5);
            robot.moveAngle(2,39.5,0.7);
            robot.turnInPlace(0,1.0);
            robot.moveAngle(39,2,0.7);
            robot.turnInPlace(2,1.0);
        }
        robot.shootRightPowerShot(3);
        if(rings!=4) {

            if (rings == 0) {
                robot.turnInPlace(0, 1.0);
                robot.moveAngle(10, 45.5, 0.7);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
                sleep(250);
                robot.turnInPlace(0, 0.5);
                robot.moveAngle(-15, 0, 0.5);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.RAISE);
                robot.moveAngle(-35, -55, 0.5);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.DROP);
                sleep(1500);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
                robot.moveAngle(15, 0, 0.7);
            }
            if (rings == 1) {
                robot.turnInPlace(0, 1.0);
                robot.moveAngle(15, 45, 0.8);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
                sleep(250);
                robot.turnInPlace(0, 0.5);
                robot.moveAngle(-15, 0, 0.5);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.RAISE);
                robot.moveAngle(-10, -82, 0.5);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.DROP);
                robot.moveAngle(5,0,0.25);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
                robot.moveAngle(7, 5, 0.7);
                robot.moveAngle(0, 15, 0.7);

            }
        }
        else{
            robot.moveAngle(0,-15,0.5);
        }
        robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
        stop();
    }



}
