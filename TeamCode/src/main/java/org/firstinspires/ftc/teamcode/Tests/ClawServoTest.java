package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ClawServoTest extends LinearOpMode{
    public static double a = 0.0;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo claw = hardwareMap.get(Servo.class, "claw");
        while(opModeIsActive()) {
            claw.setPosition(a);
        }
    }
}
