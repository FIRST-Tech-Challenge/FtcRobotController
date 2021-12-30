package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test CV Hub Alignment", group="competition")
public class TestCVHubAlignment extends LinearOpMode {
    RobotClass robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass(hardwareMap,telemetry,this);
        robot.openCVInnitShenanigans();
        FreightFrenzyComputerVision.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;
        FreightFrenzyComputerVision.SkystoneDeterminationPipeline.HubPosition hubLocation = null;
        freightLocation = robot.analyze();
        hubLocation = robot.analyze_hub();

        waitForStart();

        while (hubLocation != FreightFrenzyComputerVision.SkystoneDeterminationPipeline.HubPosition.CENTER) {
            if (hubLocation == FreightFrenzyComputerVision.SkystoneDeterminationPipeline.HubPosition.LEFT) {
                robot.frontRight.setPower(.15);
                robot.frontLeft.setPower(-.15);
                robot.backLeft.setPower(.15);
                robot.backRight.setPower(-.15);
            } else if (hubLocation == FreightFrenzyComputerVision.SkystoneDeterminationPipeline.HubPosition.RIGHT) {
                robot.frontRight.setPower(-.15);
                robot.frontLeft.setPower(.15);
                robot.backLeft.setPower(-.15);
                robot.backRight.setPower(.15);
            }
            hubLocation = robot.analyze_hub();
        }

        robot.stopMotors();
        robot.pause(3000);
        robot.pauseButInSecondsForThePlebeians(7);
    }
}
