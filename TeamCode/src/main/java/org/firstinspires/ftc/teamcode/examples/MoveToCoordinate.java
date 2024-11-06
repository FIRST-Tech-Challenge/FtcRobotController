package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.kdrobot.KDRobot;
import java.lang.Math;

@TeleOp
//@Disabled
public class MoveToCoordinate extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean robotMovedForward = false;
        boolean robotInMotion = false;
        double correction = 0;
        double leftSpeed = 0;
        double rightSpeed = 0;
        double targetY = 1000;
        double targetX = 50;
        double turnDirection = 0;
        double driveTrainSpeed = 0;
        boolean yawAngleUpdated = false;

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        KDRobot robot = new KDRobot();
        robot.init(hardwareMap, telemetry);
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double startingPosition = backLeft.getCurrentPosition();
        waitForStart();

        while (opModeIsActive()) {
            double YawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double driveTrainPoistion = backLeft.getCurrentPosition();
            YawAngle = (double) Math.round(YawAngle * 10) / 10.0;
            driveTrainSpeed = 0.15;  

            telemetry.addData("yawAngle",YawAngle);
            telemetry.addData("pos",driveTrainPoistion);
            telemetry.update();
            if (targetY >= driveTrainPoistion) {
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = -15 + correction;
                rightSpeed = -15 -correction;
                robot.setWheelPower(leftSpeed/-100,rightSpeed/-100,leftSpeed/-100,rightSpeed/-100);
            }
            else {
                robot.stopDriveTrain();
                robotMovedForward = true;
            }
            if (targetX < 0) {
                turnDirection = -1;
            } else if (targetX > 0) {
                turnDirection = 1;
            }
            if (turnDirection == -1) {
                YawAngle = YawAngle * 1;
                driveTrainSpeed = driveTrainSpeed * 1;
                yawAngleUpdated = true;
            }
            if (turnDirection == 1) {
                YawAngle = YawAngle * -1;
                driveTrainSpeed = driveTrainSpeed * -1;
                yawAngleUpdated = true;
            }
            if (yawAngleUpdated && robotMovedForward) {
                if (YawAngle >= 90) {
                    robot.stopDriveTrain();
                    telemetry.addData("yawAngle > 90","STOP ACTION");
                    telemetry.update();
                } else if (YawAngle <= 85){
                    robot.setWheelPower(-driveTrainSpeed, driveTrainSpeed, -driveTrainSpeed,driveTrainSpeed);
                    telemetry.addData("keep rotating",YawAngle);
                    telemetry.update();
                }
            }
        }

    }

}
