package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@TeleOp(name="TeleopCenterStage", group="Linear OpMode")
public class TeleopCenterStage extends LinearOpMode {

    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);
        double x, y;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            x = gamepad1. left_stick_x;
            y = -gamepad1. left_stick_y;

            double turn = gamepad1.right_stick_x;

            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math. PI/4);
            double cos = Math.cos (theta - Math. PI/4);
            double max = Math.max (Math.abs(sin),
                    Math.abs(cos));

            double leftFrontVel = power * cos/max + turn;
            double rightFrontVel = power * sin/max - turn;
            double leftRearVel = power * sin/max + turn;
            double rightRearVel = power * sin/max - turn;

            if ((power + Math.abs(turn)) > 1) {
                leftFrontVel   /= power + Math.abs(turn);
                rightFrontVel /= power + Math.abs(turn);
                leftRearVel    /= power + Math.abs(turn);
                rightRearVel  /= power + Math.abs(turn);
            }

            leftFront.setPower(leftFrontVel);
            leftRear.setPower(leftRearVel);
            rightFront.setPower(rightFrontVel);
            rightRear.setPower(rightRearVel);
        }
    }
}
