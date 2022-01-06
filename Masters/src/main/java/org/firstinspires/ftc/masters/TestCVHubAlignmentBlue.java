package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test CV Hub Alignment blue", group="test")
public class TestCVHubAlignmentBlue extends LinearOpMode {
    RobotClass robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass(hardwareMap,telemetry,this);
        robot.openCVInnitShenanigans("blue");
        FreightFrenzyComputerVisionBlueHub.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;
        FreightFrenzyComputerVisionBlueHub.SkystoneDeterminationPipeline.HubPosition hubLocation = null;
        freightLocation = robot.analyze();
        hubLocation = robot.analyze_hub_blue();

        waitForStart();

        while (hubLocation != FreightFrenzyComputerVisionBlueHub.SkystoneDeterminationPipeline.HubPosition.CENTER && opModeIsActive()) {
            if (hubLocation == FreightFrenzyComputerVisionBlueHub.SkystoneDeterminationPipeline.HubPosition.LEFT) {
//                robot.frontRight.setPower(.15);
//                robot.frontLeft.setPower(-.15);
//                robot.backLeft.setPower(.15);
//                robot.backRight.setPower(-.15);
            } else if (hubLocation == FreightFrenzyComputerVisionBlueHub.SkystoneDeterminationPipeline.HubPosition.RIGHT) {
//                robot.frontRight.setPower(-.15);
//                robot.frontLeft.setPower(.15);
//                robot.backLeft.setPower(-.15);
//                robot.backRight.setPower(.15);
            }
            hubLocation = robot.analyze_hub_blue();
        }

        robot.stopMotors();
        robot.pause(3000);
        robot.pauseButInSecondsForThePlebeians(7);
    }
}
