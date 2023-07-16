package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Lift;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFGamepad;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
@Config
@Disabled
@TeleOp(name = "LiftTest")

public class LiftTest extends LinearOpMode {
    public void runOpMode(){
        PwPRobot robot = new PwPRobot(this, true);
        RFGamepad liftGp = new RFGamepad();
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a||Lift.LiftConstants.LIFT_GROUND_JUNCTION.getLfcValue()){
                if(!Lift.LiftConstants.LIFT_GROUND_JUNCTION.getLfcValue()) {
                    robot.resetQueuer();
                    robot.setFirstLoop(true);
                    robot.liftToPosition(Lift.LiftConstants.LIFT_GROUND_JUNCTION);
                    robot.setFirstLoop(false);
                }
                else{
                    robot.liftToPosition(Lift.LiftConstants.LIFT_GROUND_JUNCTION);
                }
            }
            if(gamepad1.b||Lift.LiftConstants.LIFT_LOW_JUNCTION.getLfcValue()){
                if(!Lift.LiftConstants.LIFT_LOW_JUNCTION.getLfcValue()) {
                    robot.resetQueuer();
                    robot.setFirstLoop(true);
                    robot.liftToPosition(Lift.LiftConstants.LIFT_LOW_JUNCTION);
                    robot.setFirstLoop(false);
                }
                else{
                    robot.liftToPosition(Lift.LiftConstants.LIFT_LOW_JUNCTION);
                }
            }
            if(gamepad1.y||Lift.LiftConstants.LIFT_MED_JUNCTION.getLfcValue()){
                if(!Lift.LiftConstants.LIFT_MED_JUNCTION.getLfcValue()) {
                    robot.resetQueuer();
                    robot.setFirstLoop(true);
                    robot.liftToPosition(Lift.LiftConstants.LIFT_MED_JUNCTION);
                    robot.setFirstLoop(false);
                }
                else{
                    robot.liftToPosition(Lift.LiftConstants.LIFT_MED_JUNCTION);
                }
            }
            if(gamepad1.x||Lift.LiftConstants.LIFT_HIGH_JUNCTION.getLfcValue()){
                if(!Lift.LiftConstants.LIFT_HIGH_JUNCTION.getLfcValue()) {
                    robot.resetQueuer();
                    robot.setFirstLoop(true);
                    robot.liftToPosition(Lift.LiftConstants.LIFT_HIGH_JUNCTION);
                    robot.setFirstLoop(false);
                }
                else{
                    robot.liftToPosition(Lift.LiftConstants.LIFT_HIGH_JUNCTION);
                }
            }
            if(gamepad1.dpad_down){
                robot.liftToPosition(0);
            }
            if(gamepad1.dpad_right){
                robot.liftToPosition(500);
            }
            if(gamepad1.dpad_up){
                robot.liftToPosition(1000);
            }
            if(gamepad1.dpad_left){
                robot.liftToPosition(1400);
            }
            if(gamepad1.right_trigger!=0||gamepad1.left_trigger!=0){
                robot.setLiftPower((gamepad1.right_trigger-gamepad1.left_trigger)*0.6);
            }else{
                robot.setLiftPower(0);
            }
            liftGp.readGamepad(gamepad1.a, "gamepad1_a", "Lift to ground junction");
            liftGp.readGamepad(gamepad1.b, "gamepad1_b", "Lift to low junction");
            liftGp.readGamepad(gamepad1.y, "gamepad1_y", "Lift to medium junction");
            liftGp.readGamepad(gamepad1.x, "gamepad1_x", "Lift to high junction");
            liftGp.readGamepad(gamepad1.dpad_down, "gamepad1_dpad_down", "Lift to 0 ticks");
            liftGp.readGamepad(gamepad1.dpad_right, "gamepad1_dpad_right", "Lift to 1000 ticks");
            liftGp.readGamepad(gamepad1.dpad_up, "gamepad1_dpad_up", "Lift to 2000 ticks");
            liftGp.readGamepad(gamepad1.dpad_left, "gamepad1_dpad_left", "Lift to 3000 ticks");
            liftGp.readGamepad(gamepad1.right_trigger, "gamepad1_right_trigger", "Manual lift going up");
            liftGp.readGamepad(gamepad1.left_trigger, "gamepad1_left_trigger", "Manual lift going down");
            telemetry.update();
        }
    }
}
