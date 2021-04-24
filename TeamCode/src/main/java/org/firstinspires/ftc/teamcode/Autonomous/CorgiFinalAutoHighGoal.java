package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "CorgiFinalAutoHighGoal", preselectTeleOp = "OneGPTeleop")

public class CorgiFinalAutoHighGoal extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        int rings = robot.getRingsAndWaitForStart();
        robot.stopRingDetection();
//        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
        telemetry.addData("ReadyToStart",0);
        telemetry.update();
        if(rings!=1&&rings!=4) {
        // 0 RING AUTO
            robot.goToPosition(-54,4,0,0.8);
            robot.shootHighGoal(3);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
            robot.goToPosition(-58,-15,0,1);
            robot.openWobbleGoalClaw();
            sleep(100);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
            robot.turnInPlace(180,0.7);
            robot.goToPosition(-30,9,180,1);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
            robot.turnInPlace(180,0.7);
            robot.goToPosition(-22.5,8.5,178,0.9);
            robot.closeWobbleGoalClaw();
            sleep(250);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
            sleep(100);
            robot.turnInPlace(0,0.7);
            robot.goToPosition(-58, -12,0,0.5);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.openWobbleGoalClaw();
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            robot.goToPosition(-64, -12,0,0.5);
        }
        else if(rings==1) {
        // 1 RING AUTO
            robot.goToPosition(-55,-16,0,0.95);
            robot.goToPosition(-57,4,0,0.75);
            robot.shootHighGoal(1);
            sleep(500);
            robot.startIntake();
            robot.startTransfer();
            robot.goToPosition(-47,4,0.5,0.6);
            sleep(250);
            robot.goToPosition(-57,4,0,0.75);
            robot.shootHighGoal(3);
            robot.stopIntake();
            robot.stopTransfer();
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
            robot.goToPosition(-87,6,0,1);
            robot.openWobbleGoalClaw();
            sleep(100);
            robot.turnInPlace(180,0.7);
            robot.goToPosition(-30,9,180,1);
            robot.turnInPlace(180,0.7);
            robot.goToPosition(-22.5,8.5,178,0.9);
            robot.closeWobbleGoalClaw();
            sleep(250);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
            sleep(100);
            robot.turnInPlace(0,0.7);
            robot.goToPosition(-74,3.5,0,0.6);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.openWobbleGoalClaw();
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
        }
        else if(rings==4) {
        // 4 RING AUTO
            robot.goToPosition(-55,-16,0,0.95);
            robot.goToPosition(-106,-16,0,0.9);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            sleep(250);
            robot.openWobbleGoalClaw();
            sleep(50);
            robot.setVelocity(1650,1000);
            robot.goToPosition(-55.5,4,0,0.6);
            robot.shootHighGoal(3);
            sleep(100);
            robot.startIntake();
            robot.startTransfer();
            robot.goToPosition(-49,3.5,0.5,0.8);
            sleep(250);
            robot.goToPosition(-55.5, 3, 0, 0.8);
            robot.shootHighGoal(1);
            sleep(100);
            robot.goToPosition(-27,4,0,0.8);//40
            sleep(250);
            robot.goToPosition(-56.5,2.5,0,0.8);
            robot.shootHighGoal(3);
            robot.stopIntake();
            robot.stopTransfer();
            robot.turnInPlace(180,0.7);
            robot.goToPosition(-30,9,180,1);
            robot.turnInPlace(180,0.7);
            robot.goToPosition(-22.5,8.5,172.5,0.9);
            robot.closeWobbleGoalClaw();
            sleep(250);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
            sleep(100);
            robot.turnInPlace(0,0.7);
            robot.goToPosition(-104, -8.5,0,1);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.openWobbleGoalClaw();
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(50);
            robot.goToPosition(-74, -8.5,0,1);
        }
        stop();
    }
}
