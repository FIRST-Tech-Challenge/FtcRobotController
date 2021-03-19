package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.OdometryChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "CorgiFinalAutoHighGoal", preselectTeleOp = "OneGPTeleop")

public class CorgiFinalAutoHighGoal extends LinearOpMode {
    @Override
    public void runOpMode(){
        OdometryChassis robot = new OdometryChassis(this,false,true);
        ElapsedTime runtime = new ElapsedTime();
//        int rings = robot.getRingsAndWaitForStart();
//        robot.navigate();
//        robot.stopRingDetection();
//        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
        waitForStart();
//        if(rings!=1&&rings!=4) {
        // 0 RING AUTO
            robot.goToPosition(23,10,-13.5,0.9);//high goal shoot area
//            robot.shootHighGoal(3);
            robot.goToPosition(75,10,0,1);
//            robot.openWobbleGoalClaw();
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.goToPosition(8,-40,0,0.8);
            robot.turnInPlace(90, 0.6);
            robot.goToPosition(8,-30,90,0.6);
//            robot.closeWobbleGoalClaw();
            sleep(600);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(70,15,0,0.6);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
//            robot.openWobbleGoalClaw();
            sleep(250);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
//        }
//        else if(rings==1) {
        // 1 RING AUTO
//            robot.goToPosition(-23,-10,-13.5,0.9);
//            robot.shootHighGoal(3);
////            robot.shootHighGoalTest(1670, 100,3);
//            robot.goToPosition(-96,14,0,1);
//            robot.openWobbleGoalClaw();
//            robot.goToPosition(-60,4,1,0.6);
//            robot.startIntake();
//            robot.startTransfer();
//            robot.turnInPlace(0,1.0);
//            robot.goToPosition(-45,7,0.5,0.5);
//            sleep(250);
//            robot.shootHighGoal(1);
//            sleep(250);
//            robot.stopIntake();
//            robot.stopTransfer();
//            sleep(250);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
//            robot.turnInPlace(0, 1.0);
//            robot.goToPosition(-5,39,0,0.8);
//            robot.turnInPlace(0,0.6);
//            robot.goToPosition(-2,28,0,0.8);
//            robot.closeWobbleGoalClaw();
//            sleep(600);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
//            robot.goToPosition(-91,14,0,0.8);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
//            robot.openWobbleGoalClaw();
//            sleep(250);
//            robot.goToPosition(-74,17.5,0,0.6);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
//            robot.turnInPlace(180,0.7);
//        }
//        else if(rings==4) {
        // 4 RING AUTO
//            robot.goToPosition(-23,-10,-13.5,0.9);
//            robot.shootHighGoalTest(1670, 100,3);
//            robot.goToPosition(-115, -7, 0,1);
//            robot.openWobbleGoalClaw();
//            robot.goToPosition(-60,-2,1,0.6);
//            robot.startIntake();
//            robot.startTransfer();
//            robot.goToPosition(-45,-1,0.5,0.5);
//            robot.goToPosition(-43,0,-2,0.4);
//            sleep(500);
//            robot.shootHighGoal(1);
////            robot.startIntake();
////            robot.startTransfer();
//            sleep(250);
//            robot.goToPosition(-33,0,0,1);
//            robot.goToPosition(-33,0,-2,1);
//            robot.shootHighGoalTest(1670, 72,3);
//            robot.stopIntake();
//            robot.stopTransfer();
//            sleep(250);
//            robot.goToPosition(-70,16, 0,1);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
//            robot.goToPosition(-70,16, 180,1);
//            sleep(250);
//        }
        stop();
    }
}
