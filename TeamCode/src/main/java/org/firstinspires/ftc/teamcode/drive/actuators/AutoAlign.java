package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum AutoAlign", group = "Linear Opmode")
public class AutoAlign extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private IMU imu;

    private double referenceHeading = 0;
    private boolean imuResetPressed = false;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "odol");
        leftBack = hardwareMap.get(DcMotor.class, "odor");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        rightBack = hardwareMap.get(DcMotor.class, "odom");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {

            double lx = -gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx;

            double drivePower = 1 - (0.5 * gamepad1.right_trigger);

            // Atualiza referência de heading com right bumper
            if (gamepad1.right_bumper) {
                referenceHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            // Reset de yaw com left trigger
            if (gamepad1.left_trigger > 0.5) {
                if (!imuResetPressed) {
                    imu.resetYaw();
                    imuResetPressed = true;
                }
            } else {
                imuResetPressed = false;
            }

            // heading atual
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Cálculo de rotação
            if (gamepad1.left_bumper) {
                // Enquanto estiver pressionado, rotaciona para alinhar com o basket
                double headingError = angleWrap(referenceHeading - heading);
                rx = headingError * 1.5; // constante P
            } else {
                // Senão, usa o stick direito normalmente
                rx = -gamepad1.right_stick_x;
            }

            // Field-centric transform
            double adjustedLx = ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) - lx * Math.sin(heading);

            double max = Math.max(Math.abs(adjustedLy) + Math.abs(adjustedLx) + Math.abs(rx), 1.0);

            leftFront.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
            leftBack.setPower(((adjustedLy - adjustedLx + rx) / max) * drivePower);
            rightFront.setPower(((adjustedLy - adjustedLx - rx) / max) * drivePower);
            rightBack.setPower(((adjustedLy + adjustedLx - rx) / max) * drivePower);

            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.addData("Target", Math.toDegrees(referenceHeading));
            telemetry.addData("Align Active", gamepad1.left_bumper);
            telemetry.update();
        }
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
