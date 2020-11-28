package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.Odometry;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name= "R_Power_park")
public class R_Power_park extends LinearOpMode {
    final boolean debug = true;

    @Override
    public void runOpMode(){
        int rings=-1, i=0;
        ElapsedTime runtime = new ElapsedTime();;
        Robot robot=new Robot(this, BasicChassis.ChassisType.IMU);
        robot.initTensorFlow();
        while (runtime.seconds()<4) {
            robot.runTensorFlow();
            sleep(50);
            rings = robot.tensorFlow.getNumberOfRings();
            telemetry.addData("Number of Rings: ", "i=%d %d", i++, rings);
        }
        telemetry.update();
        waitForStart();
        rings = robot.tensorFlow.getNumberOfRings();
        robot.stopTensorFlow();
        /*robot.moveAngle(0,-60,0.5);
        robot.shootPowerShot(3);
        robot.moveAngle(0,-14,0.5);
        sleep(500);*/
        robot.moveWobbleGoalServo(false);
        rings=0;
        if(rings==0) {
            robot.moveAngle( -40,-58, 0.7);
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(0,10,0.8);
            robot.turnInPlace(0,1.0);
            robot.moveAngle(35,2,0.5);
            robot.turnInPlace(0,1.0);
        }
        else if(rings==1) {
            robot.moveAngle(5,-83, 0.7);
            robot.turnInPlace(0,0.5);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(0,10, 0.7);
            robot.turnInPlace(0,0.5);
            robot.moveAngle(14,19, 0.7);
            robot.turnInPlace(0,1.0);
        }
        else if(rings==4) {
            robot.moveAngle(-40, -104,0.7);
            robot.turnInPlace(0,0.5);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(-0, 10,0.7);
            robot.turnInPlace(0,0.5);
            robot.moveAngle(37,45.5,0.7);
            robot.turnInPlace(0,1.0);
        }
        robot.shootRightPowerShot(3);
        robot.moveAngle(0,-10,0.8);
        sleep(500);
        stop();
    }




}
