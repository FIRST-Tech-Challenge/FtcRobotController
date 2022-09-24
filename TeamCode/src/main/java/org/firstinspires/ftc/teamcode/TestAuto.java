package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.bots.SnarmBot;

@Autonomous(name="Test Robot", group="Tests")

public class TestAuto extends LinearOpMode {

    protected FSMBot robot = new FSMBot(this); //replace FourWheelDriveBot with whichever Bot is required

    @Override
    public void runOpMode() {

        robot.snarmSnarmState = 1;
        robot.isAutonomous = false;
        robot.init(hardwareMap);
        robot.bypassAfterReadyAgain = false;
        robot.goToIntakePosition(3);
        waitForStart();
        robot.snarmState = SnarmBot.SnarmState.AFTER_READY_AGAIN;
        robot.goToIntakePosition(3);
        //move flipper
        robot.goToFlipperPosition(5);
        robot.sleep(1000);
        //extension to 700
        robot.setExtension(700);
        robot.sleep(1000);
        //elevation to 0.66
        robot.setElevationPosition(0.66);
        robot.sleep(1000);
        //rotate to blue shared
        robot.setRotationPosition(robot.blueSharedRotation);
        robot.sleep(2000);
        //open close box
        robot.box.setPosition(robot.boxOpened);
        robot.sleep(300);
        robot.box.setPosition(robot.boxLocked);
        robot.sleep(300);
        //rotate to red shared
        robot.setRotationPosition(robot.redSharedRotation);
        robot.sleep(4000);
        //rotate back to center
        robot.setRotationPosition(robot.rotationCenter);
        robot.sleep(2000);
        //elevation to intaking height
        robot.setElevationPosition(robot.snarmIntakingHeight);
        robot.sleep(1000);
        //retract slide
        robot.setExtension(robot.minExtension);
        robot.sleep(1000);
        //start rotation
        robot.startRotation();
        robot.sleep(500);
        //raise intake
        robot.goToIntakePosition(2);
        robot.sleep(1000);
        //stop rotation
        robot.stopRotation();
        robot.sleep(500);
        //tape measure max elevation
        robot.setElevation(1);
        robot.sleep(500);
        //tape measure extend
        robot.controlCoreHex(1, 0);
        robot.sleep(1000);
        robot.controlCoreHex(0, 0);
        //tape measure min elevation
        robot.setElevation(0);
        robot.sleep(500);
        //tape measure retract
        robot.controlCoreHex(0, 1);
        robot.sleep(700);
        robot.setElevation(0.3);
        robot.sleep(700);
        robot.controlCoreHex(0, 0);
        //lower odometry
        robot.odometryRaise.setPosition(0.65);
        robot.sleep(500);
        //raise odometry
        robot.odometryRaise.setPosition(0.88);
        robot.sleep(500);
        //lower odometry
        robot.odometryRaise.setPosition(0.65);
        robot.sleep(500);
        //raise odometry
        robot.odometryRaise.setPosition(0.88);
        robot.sleep(500);
        //duck spinner blue
        robot.toggleSpinner(0.27, false);
        robot.sleep(2000);
        robot.toggleSpinner(0.27, false);
        robot.sleep(1000);
        //duck spinner red
        robot.toggleSpinner(0.27, true);
        robot.sleep(2000);
        robot.toggleSpinner(0.27, true);
        robot.sleep(1000);
    }
}
