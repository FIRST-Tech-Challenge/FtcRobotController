package org.firstinspires.ftc.teamcode.opmodes.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.robot.BioBuzzRobot;

@TeleOp
public class DrivetrainTesting extends LinearOpMode {
    @Override
    public void runOpMode(){
        BioBuzzRobot robot = new BioBuzzRobot(hardwareMap, telemetry);
        robot.initialize();
        robot.getDrivetrain().enableFieldCentricDriving(false);
        waitForStart();
        while(opModeIsActive()){
            robot.getDrivetrain().updateMovementVector(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x);
            robot.teleopUpdate();
        }
    }
}
