package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

//notes: why after collision goes back to robot centric?

@TeleOp(name = "fieldCentric")
public class LinearTeleOp_fieldCentric extends Base {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        waitForStart();

        if (isStopRequested()) return;

        boolean lastMovementLauncher = false;
        boolean toggleMovementLauncher = false;

        boolean lastMovementWrist = false;
        boolean toggleMovementWrist = false;

        boolean lastMovementCL = false;
        boolean toggleMovementCL = false;

        boolean lastMovementCR = false;
        boolean toggleMovementCR = false;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double topLeftPow = (rotY + rotX + rx) / denominator;
            double backLeftPow = (rotY - rotX + rx) / denominator;
            double topRightPow = (rotY - rotX - rx) / denominator;
            double backRightPow = (rotY + rotX - rx) / denominator;

            topLeft.setPower(topLeftPow);
            backLeft.setPower(backLeftPow);
            topRight.setPower(topRightPow);
            backRight.setPower(backRightPow);

            arm1.setDirection(DcMotorSimple.Direction.REVERSE);
            double armPower = (-.2 * gamepad1.left_trigger + .35 * gamepad1.right_trigger);
            arm1.setPower(armPower);
            arm2.setPower(armPower);


            planeLauncher.setDirection(Servo.Direction.REVERSE);
            boolean launcherCurrentMovement = gamepad1.a;
            if (launcherCurrentMovement && !lastMovementLauncher) {
                toggleMovementLauncher = !toggleMovementLauncher;
                if (toggleMovementLauncher) {
                    planeLauncher.setPosition(0);
                } else {
                    planeLauncher.setPosition(.7);
                }
            }
            lastMovementLauncher = launcherCurrentMovement;

            boolean leftClawCurrentMovement = gamepad1.left_bumper;
            if (leftClawCurrentMovement && !lastMovementCL) {
                toggleMovementCL = !toggleMovementCL;
                if (toggleMovementCL) {
                    clawLeft.setPosition(.15); // close position
                } else {
                    clawLeft.setPosition(.35);
                }
            }
            lastMovementCL = leftClawCurrentMovement;

            boolean rightClawCurrentMovement = gamepad1.right_bumper;
            if (rightClawCurrentMovement && !lastMovementCR) {
                toggleMovementCR = !toggleMovementCR;
                if (toggleMovementCR) {
                    clawRight.setPosition(.65);
                } else {
                    clawRight.setPosition(.85); // close position
                }
            }
            lastMovementCR = rightClawCurrentMovement;

            boolean toggleWrist = gamepad1.x;
            if (toggleWrist && !lastMovementWrist) {
                toggleMovementWrist = !toggleMovementWrist;
                if (toggleMovementWrist) {
                    clawAngle.setPosition(0);
                } else {
                    clawAngle.setPosition(1);
                }
            }
            lastMovementWrist = toggleWrist;
        }
    }
}
