package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.Names;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "Debugging Teleop", group = "DEBUG")
public class DebuggingTeleop extends OpMode {


    Follower follower;

    static DcMotorEx slideLeft;
    static DcMotorEx slideRight;

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
        slideLeft = hardwareMap.get(DcMotorEx.class, Names.SLIDE_MOTOR_LEFT);
        slideRight = hardwareMap.get(DcMotorEx.class, Names.SLIDE_MOTOR_RIGHT);

    }

    @Override
    public void loop() {

        slideLeft.setPower(gamepad1.left_stick_y);
        slideRight.setPower(gamepad1.left_stick_y);

        telemetry.addData("Slide Left Position ", slideLeft.getCurrentPosition());
        telemetry.addData("Slide Right Position ", slideRight.getCurrentPosition());
        telemetry.addData("X ", follower.getPose().getX());
        telemetry.addData("Y ", follower.getPose().getY());
        telemetry.addData("Heading (Deg) ", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("Left Encoder ", slideLeft.getCurrentPosition());
        telemetry.addData("Right Encoder ", slideRight.getCurrentPosition());
        telemetry.addData("Current Left (MA) ", slideLeft.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Current Right (MA) ", slideRight.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Current Read Position ", getCurrentPosition());



    }

    public static int getCurrentPosition(){
        return (int) ((double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2);
    }
}
