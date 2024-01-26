package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "EncodersTesting", group = "Tests")
public class IMUEmmulatedTuning extends LinearOpMode {
    DcMotor leftEncoder, rightEncoder;

    int leftPos, rightPos, diff;
    double imuRaw, proportion = 0.003237, imuEmmulated;

    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = hardwareMap.get(DcMotor.class, "rearRight");
        rightEncoder = hardwareMap.get(DcMotor.class, "frontLeft");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();

        while(opModeIsActive()) {
            leftPos = leftEncoder.getCurrentPosition();
            rightPos = rightEncoder.getCurrentPosition();
            diff = rightPos-leftPos;
            imuRaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            proportion = imuRaw/diff;
            imuEmmulated = diff*proportion;

            telemetry.addData("Left Position: ", leftPos);
            telemetry.addData("Right Position: ", rightPos);
//            telemetry.addData("Proportion: ", proportion);
//            telemetry.addData("Diff: ", diff);
            telemetry.addData("Yaw: ", imuRaw);
            telemetry.addData("Emmulated Yaw: ", imuEmmulated);
            telemetry.update();
        }
    }
}
