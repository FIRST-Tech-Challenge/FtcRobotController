package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name ="TeleOp")
public class TeleOp extends LinearOpMode {
    private GamepadEvents controller;
    private MechDrive robot;
    private Limelight limelight;
    private Imu imu;



    public void runOpMode() throws InterruptedException{

        controller = new GamepadEvents(gamepad1);
        robot = new MechDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        imu = new Imu(hardwareMap);



        waitForStart();
        while(opModeIsActive())
        {

            double forward = -controller.left_stick_y;
            double strafe = -controller.left_stick_x;
            double rotate = -controller.right_stick_x;

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            LLResult result = limelight.getLatestResult();

            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.update();
                }
            }

            robot.drive(forward, strafe, rotate);
            controller.update();
        }


    }

}
