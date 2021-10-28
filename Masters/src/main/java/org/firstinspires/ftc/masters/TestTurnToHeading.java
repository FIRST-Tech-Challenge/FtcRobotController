package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp
public class TestTurnToHeading extends LinearOpMode {
    RobotClass robot;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        robot.setSpeedForTurnLeft(0.5);

        telemetry.addData("Moving Forward","");
        telemetry.update();
        robot.forward(.1,.1);
        robot.stopMotors();

        telemetry.addData("Turning to: ", 90);
        telemetry.addData("Current Heading: ", robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("Integrated Heading: ", robot.getIntegratedHeading());
        robot.motorTelemetry();
        telemetry.update();
        robot.turnToHeading(.5,90,2);
        robot.pauseButInSecondsForThePlebeians(1);

        telemetry.addData("Turning to ", 180);
        telemetry.addData("Current Heading: ", robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("Integrated Heading: ", robot.getIntegratedHeading());
        robot.motorTelemetry();
        telemetry.update();
        robot.turnToHeading(.5,180,1);
        robot.pauseButInSecondsForThePlebeians(1);

        telemetry.addData("Turning to ", 90);
        telemetry.addData("Current Heading: ", robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("Integrated Heading: ", robot.getIntegratedHeading());
        robot.motorTelemetry();
        telemetry.update();
        robot.turnToHeading(.5,90,3);
        robot.pauseButInSecondsForThePlebeians(1);

        telemetry.addData("Turning to ", 270);
        telemetry.addData("Current Heading: ", robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("Integrated Heading: ", robot.getIntegratedHeading());
        robot.motorTelemetry();
        telemetry.update();
        robot.turnToHeading(.5,270,4);
        robot.pauseButInSecondsForThePlebeians(10);
    }
}
