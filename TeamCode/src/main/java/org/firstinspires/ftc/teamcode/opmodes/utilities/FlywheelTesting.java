package org.firstinspires.ftc.teamcode.opmodes.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.BioBuzzRobot;

@TeleOp
public class FlywheelTesting extends LinearOpMode {
    private static int FLYWHEEL_SPEED_RPM= 250;
    @Override
        public void runOpMode()  {
            BioBuzzRobot robot = new BioBuzzRobot(hardwareMap, telemetry );
           robot.initialize();
            waitForStart();

            while(opModeIsActive()){
                if(gamepad1.circle){
                    robot.getFlywheelPollen().setTargetSpeed(FLYWHEEL_SPEED_RPM);
                }else {
                    robot.getFlywheelPollen().setTargetSpeed(0);
                }
            }

        }
    }