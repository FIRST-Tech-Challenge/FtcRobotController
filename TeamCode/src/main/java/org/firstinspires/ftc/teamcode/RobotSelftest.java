package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.toDegrees;
import static java.lang.Thread.sleep;

/**
 * Created by Ethan on 12/2/2016.
 */

//@TeleOp(name="Robot Selftest", group ="Test")
public class RobotSelftest extends LinearOpMode {

    public HardwareOmnibot robot = new HardwareOmnibot();

    boolean runOnce = true;
    final double SCALE_FACTOR = 255;
    @Override
    public void runOpMode() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        robot.init(hardwareMap);
        robot.setInputShaping(false);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        do {
            int leftForeEncoderStart = robot.frontLeft.getCurrentPosition();
            int rightForeEncoderStart = robot.frontRight.getCurrentPosition();
            int rightRearEncoderStart = robot.rearRight.getCurrentPosition();
            int leftRearEncoderStart = robot.rearLeft.getCurrentPosition();
            robot.setAllDrive(0.1);
            sleep(500);
            robot.setAllDriveZero();
            int leftForeEncoder = robot.frontLeft.getCurrentPosition();
            int rightForeEncoder = robot.frontRight.getCurrentPosition();
            int rightRearEncoder = robot.rearRight.getCurrentPosition();
            int leftRearEncoder = robot.rearLeft.getCurrentPosition();
            robot.setAllDrive(-0.1);
            sleep(500);
            robot.setAllDriveZero();
            int leftForeEncoderEnd = robot.frontLeft.getCurrentPosition();
            int rightForeEncoderEnd = robot.frontRight.getCurrentPosition();
            int rightRearEncoderEnd = robot.rearRight.getCurrentPosition();
            int leftRearEncoderEnd = robot.rearLeft.getCurrentPosition();
            telemetry.addData("Left Fore Count", leftForeEncoder);
            telemetry.addData("Right Fore Count", rightForeEncoder);
            telemetry.addData("Right Rear Count", rightRearEncoder);
            telemetry.addData("Left Rear Count", leftRearEncoder);

//            int extenderEncoderStart = robot.extender.getCurrentPosition();
//            robot.setExtenderMotorPower(0.1, true);
            sleep(500);
//            robot.setExtenderMotorPower(0.0, true);
//            int extenderEncoder = robot.extender.getCurrentPosition();
//            telemetry.addData("Extender Count", extenderEncoder);

//            int leftRotatorEncoderStart = robot.rotator1.getCurrentPosition();
//            int rightRotatorEncoderStart = robot.rotator2.getCurrentPosition();
//            robot.setRotatorMotorPower(0.2);
            sleep(500);
//            robot.setRotatorMotorPower(0.0);
//            int leftRotatorEncoder = robot.rotator1.getCurrentPosition();
//            int rightRotatorEncoder = robot.rotator2.getCurrentPosition();
//            robot.setRotatorMotorPower(-0.2);
            sleep(500);
//            robot.setRotatorMotorPower(0.0);
//            int leftRotatorEncoderEnd = robot.rotator1.getCurrentPosition();
//            int rightRotatorEncoderEnd = robot.rotator2.getCurrentPosition();

//            telemetry.addData("Left Rotator Count", leftRotatorEncoder);
//            telemetry.addData("Right Rotator Count", rightRotatorEncoder);

//            robot.setExtenderMotorPower(-0.1, true);
            sleep(500);
//            robot.setExtenderMotorPower(0.0, true);
//            int extenderEncoderEnd = robot.extender.getCurrentPosition();

            float hsvValues[] = {0F, 0F, 0F};
//            int leftRed = robot.sensorColorLeft.red();
//            int leftGreen = robot.sensorColorLeft.green();
//            int leftBlue = robot.sensorColorLeft.blue();
//            Color.RGBToHSV((int) (leftRed * SCALE_FACTOR),
//                    (int) (leftGreen * SCALE_FACTOR),
//                    (int) (leftBlue * SCALE_FACTOR),
//                    hsvValues);
            double leftColorSensor = hsvValues[0];
            telemetry.addData("Left Color Hue", leftColorSensor);

//            int rightRed = robot.sensorColorRight.red();
//            int rightGreen = robot.sensorColorRight.green();
//            int rightBlue = robot.sensorColorRight.blue();
//            Color.RGBToHSV((int) (rightRed * SCALE_FACTOR),
//                    (int) (rightGreen * SCALE_FACTOR),
//                    (int) (rightBlue * SCALE_FACTOR),
//                    hsvValues);
            double rightColorSensor = hsvValues[0];
            telemetry.addData("Right Color Hue", rightColorSensor);

//            double leftDistanceSensor = robot.sensorDistanceLeft.getDistance(DistanceUnit.CM);
//            telemetry.addData("Left Distance", leftDistanceSensor);

//            double rightDistanceSensor = robot.sensorDistanceRight.getDistance(DistanceUnit.CM);
//            telemetry.addData("Right Distance", rightDistanceSensor);
            telemetry.update();
            while(!gamepad1.x && opModeIsActive()) {
                sleep(20);
            }
        } while(opModeIsActive());
        robot.stopGroundEffects();
    }
}
