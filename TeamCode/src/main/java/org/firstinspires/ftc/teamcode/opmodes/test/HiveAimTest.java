package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.subsystems.vision.TurretCamera;


@TeleOp(name="HiveAimTest", group="Linear OpMode")
public class HiveAimTest extends LinearOpMode {

        public Robot robot;

        @Override
        public void runOpMode() throws InterruptedException {
            robot = new Robot(hardwareMap);
            robot.turretcam.setAlliance(TurretCamera.Alliance.RED);
            waitForStart();

            while (opModeIsActive()){
                robot.periodic();
                robot.writeTelemetry(telemetry);
            }

        }

    }

