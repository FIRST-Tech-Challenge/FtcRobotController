package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "Debugging Teleop", group = "DEBUG")
public class DebuggingTeleop extends OpMode {


    Follower follower;

    DcMotorEx slideLeft;
    DcMotorEx slideRight;

    Servo claw;
    public static double clawTarget = 0;
    Servo clawRotation;
    public static double clawRotationTarget = 0;

    Servo linkage;
    public static double linkageTarget = 0;


    Servo outtakePivotRight;
    public static double outtakePivotRightTarget = 0;
    Servo outtakePivotLeft;
    public static double outtakePivotLeftTarget = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
    }

    @Override
    public void loop() {

        telemetry.addData("Slide Left Position ", slideLeft.getCurrentPosition());
        telemetry.addData("Slide Right Position ", slideRight.getCurrentPosition());
        telemetry.addData("X ", follower.getPose().getX());
        telemetry.addData("Y ", follower.getPose().getY());
        telemetry.addData("Heading (Deg) ", Math.toDegrees(follower.getPose().getHeading()));



    }
}
