package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.masters.CSCons;

@Config
@TeleOp(name = "Claw Angle Test: For Karine")
public class ClawAngleTest extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double clawAngleUP = 0.4;
    public static double clawAngleDOWN = 0.82;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo clawAngle = hardwareMap.servo.get("clawArm");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                clawAngle.setPosition(clawAngleUP);
            }
            if(gamepad1.dpad_down){
                clawAngle.setPosition(clawAngleDOWN);
            }

        }
    }
}