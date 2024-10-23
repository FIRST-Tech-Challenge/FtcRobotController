package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
    @TeleOp(name="MechDrive")
    public class MecanumDriveTester extends LinearOpMode {

        private DcMotor leftFront, rightFront, leftBack, rightBack;
        private MechDrive robot;
        private GamepadEvents controller;
        private IMU imu;
        @Override
        public void runOpMode() throws InterruptedException {
            controller = new GamepadEvents(gamepad1);
            robot = new MechDrive(hardwareMap);

            telemetry.addLine("Starting Soon");
            telemetry.update();
            waitForStart();

            while(opModeIsActive())
            {
                double forward = controller.left_stick_y;
                double strafe = controller.left_stick_x;
                double rotate = controller.right_stick_x;


                if(controller.a.onPress())
                {
                    robot.resetIMU();
                }


                controller.update();
            }
        }
}
