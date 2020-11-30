package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.Odometry;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;
@Disabled
@Autonomous(name= "R_3Power_park")
public class R_3Power_park extends LinearOpMode {
    final boolean debug= true;
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
        if(rings==0) {
            robot.moveAngle( -38,-58, 0.7);
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(0,10,0.8);
            robot.turnInPlace(0,1.0);
            robot.moveAngle(33,5,0.7);
            robot.turnInPlace(0,1.0);
            robot.shootRightPowerShot(3);
            robot.moveAngle(5,38,0.8);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
            sleep(1050);
            robot.moveAngle(-20,0,0.5);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.RAISE);
            sleep(250);
            robot.moveAngle(-35,-50,0.6);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
            sleep(1000);
            robot.moveAngle(10,0,0.8);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
        }
        else if(rings==1) {
            robot.moveAngle(5,-83, 0.7);
            robot.turnInPlace(0,0.5);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(0,10, 0.7);
            robot.turnInPlace(0,0.5);
            robot.moveAngle(14,19, 0.7);
            robot.shootRightPowerShot(3);
            robot.moveAngle(0,38,0.7);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
            sleep(1050);
            robot.moveAngle(-15,0,0.5);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.RAISE);
            sleep(250);
            robot.moveAngle(-20,-70,0.5);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
            sleep(1000);
            robot.moveAngle(10,0,0.5);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
            robot.moveAngle(0,20,0.7);
        }
        else if(rings==4) {
            robot.moveAngle(-40, -104,0.7);
            robot.turnInPlace(0,0.5);
            robot.moveWobbleGoalServo(true);
            robot.moveAngle(-0, 10,0.7);
            robot.turnInPlace(0,0.5);
            robot.moveAngle(37,45.5,0.7);
            robot.shootRightPowerShot(3);
            robot.moveAngle(0,-10,0.5);
        }
        sleep(500);
        stop();
    }



}
