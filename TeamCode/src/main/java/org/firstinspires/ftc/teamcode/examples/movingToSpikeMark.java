package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.kdrobot.DriveTrainDirection;
import org.firstinspires.ftc.teamcode.kdrobot.KDRobot;

@Autonomous
public class movingToSpikeMark extends LinearOpMode {
    @Override
    public void runOpMode() {

        IMU imu = hardwareMap.get(IMU.class, "imu");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        KDRobot robot = new KDRobot();
        robot.init(hardwareMap, telemetry);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        IMU.Parameters myIMUparameters;
        boolean robotMovedForward1 = false;
        boolean robotMoveForWard2 = false;
        boolean moveForward = false;
        double correction;
        double leftSpeed;
        double rightSpeed;
        double targetY = 1117;
        double turnDirection = 0;
        double driveTrainSpeed = 0;
        boolean outake = false;
        double spikeNumber = 1;
        boolean startObjectDetection = false;
        boolean endObjectDetection = false;
        double stepNumber = 1;
        waitForStart();

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(myIMUparameters);

        imu.resetYaw();
        double yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("YawAngle",yawAngle*100);
        telemetry.update();
        int control = 0;
        int position = 1;
        boolean yawAngleUpdated = false;
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("StepNumber",stepNumber);
            telemetry.update();
            double YawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double driveTrainPoistion = backLeft.getCurrentPosition();
            YawAngle = (double) Math.round(YawAngle * 10) / 10.0;
            driveTrainSpeed = 0.27;

            //telemetry.addData("yawAngle",YawAngle);
            //telemetry.addData("pos",driveTrainPosition);
            telemetry.update();
            if (500 >= driveTrainPoistion && stepNumber == 1) {
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = -25 + correction;
                rightSpeed = -25 -correction;
                robot.setWheelPower(leftSpeed/-100,rightSpeed/-100,leftSpeed/-100,rightSpeed/-100);
            }
            else if (stepNumber == 1){
                robot.stopDriveTrain();
                stepNumber = 2;
                robotMovedForward1 = true;
            }
            if (stepNumber == 2) {
                //Add code to detect team prop
                //spikeNumber = getSpikeNumber
                sleep(2500);
                stepNumber = 3;
            }
            if (stepNumber == 3) {
                stepNumber = 4;
            }
            if (1117 >= driveTrainPoistion && stepNumber == 4) {
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = -25 + correction;
                rightSpeed = -25 -correction;
                robot.setWheelPower(leftSpeed/-100,rightSpeed/-100,leftSpeed/-100,rightSpeed/-100);
            }
            else if (stepNumber == 4){
                robot.stopDriveTrain();
                stepNumber = 5;
            }
            if (spikeNumber == 1) {
                turnDirection = -1;
            } else if (spikeNumber == 3) {
                turnDirection = 1;
            }
            if (spikeNumber == 2 && stepNumber == 5) {
                stepNumber = 6;
            }
            if (turnDirection == -1 && stepNumber == 5) {
                yawAngleUpdated = true;
            }
            if (turnDirection == 1 && stepNumber == 5) {
                YawAngle = YawAngle * -1;
                driveTrainSpeed = driveTrainSpeed * -1;
                yawAngleUpdated = true;
            }
            if (yawAngleUpdated && stepNumber == 5) {
                if (YawAngle >= 89) {
                    robot.stopDriveTrain();
                    //telemetry.addData("yawAngle > 90","STOP ACTION");
                    telemetry.update();
                    stepNumber = 6;
                } else if (YawAngle <= 90){
                    robot.setWheelPower(-driveTrainSpeed, driveTrainSpeed, -driveTrainSpeed,driveTrainSpeed);
                    //telemetry.addData("keep rotating",YawAngle);
                    telemetry.update();
                }
            }
            if (stepNumber == 6) {
                robot.setDriveTrainSpeed(0.3);
                robot.moveDriveTrain(DriveTrainDirection.FORWARD);
                sleep(50);
                robot.stopDriveTrain();
                outake = true;
            }
        }
    }
}
