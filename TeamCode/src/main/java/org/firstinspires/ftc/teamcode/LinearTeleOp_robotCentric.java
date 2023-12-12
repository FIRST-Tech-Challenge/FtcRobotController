package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "robotCentric")
public class LinearTeleOp_robotCentric extends Base {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status","Initialized");
        telemetry.update();

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        boolean lastMovementLauncher = false;
        boolean toggleMovementLauncher = false;

        boolean lastMovementWrist = false;
        boolean toggleMovementWrist = false;

        boolean lastMovementCL = false;
        boolean toggleMovementCL = false;

        boolean lastMovementCR = false;
        boolean toggleMovementCR = false;

        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y * -1;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x * 1.1;

            double denominator = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);
            double topLeftPow = (drive + turn + strafe) / denominator;
            double backLeftPow = (drive + turn - strafe) / denominator;
            double topRightPow = (drive - turn - strafe) / denominator;
            double backRightPow = (drive - turn + strafe) / denominator;

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
