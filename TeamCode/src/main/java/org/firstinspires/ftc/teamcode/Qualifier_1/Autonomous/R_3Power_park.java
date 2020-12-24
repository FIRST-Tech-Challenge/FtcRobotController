package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.Odometry;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name= "R_3Power_park_1")//12.76-13.7
@Disabled
public class R_3Power_park extends LinearOpMode {
    final boolean debug= true;
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.IMU);
        ElapsedTime runtime = new ElapsedTime();

        int rings = robot.runTensorFlowWaitForStart();

//        waitForStart();
//        rings = robot.tensorFlow.getNumberOfRings();
        robot.stopTensorFlow();
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
            robot.moveAngle(0,3.5, 0.7);
            robot.turnInPlace(0,0.5);
            robot.moveAngle(14,22, 0.7);
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
            robot.moveAngle(38,2,0.7);
            robot.turnInPlace(0,1.0);
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
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.RELEASE);
                robot.moveAngle(-35, -52, 0.5);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.DROP);
                robot.moveAngle(15,0,0.5);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
                sleep(5000);
            }
            if (rings == 1) {
                robot.turnInPlace(0, 1.0);
                robot.moveAngle(15, 45, 0.8);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
                sleep(250);
                robot.turnInPlace(0, 0.5);
                robot.moveAngle(-25, 0, 0.5);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.RELEASE);
                robot.moveAngle(-5, -82, 0.5);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.DROP);
                robot.moveAngle(10,0,0.4);
                robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
                robot.moveAngle(7, 30, 0.7);

            }
        }
        else{
            robot.moveAngle(0,-15,0.5);
        }
        robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
        stop();
    }



}
