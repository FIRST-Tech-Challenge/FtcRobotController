package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Arm Drive", group="Linear OpMode")
//@Disabled
public class ArmDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor airplaneLauncher = null;
    private DcMotor armBase = null;

    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "drive_front_left");
        leftBackDrive = hardwareMap.get(DcMotor.class, "drive_back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "drive_front_right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "drive_back_right");
        airplaneLauncher = hardwareMap.get(DcMotor.class, "airplane_launcher");
        armBase = hardwareMap.get(DcMotor.class, "arm_base");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armBase.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                airplaneLauncher.setTargetPosition(71);
                airplaneLauncher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                airplaneLauncher.setPower(1.5);
            } else {
                airplaneLauncher.setTargetPosition(0);
                airplaneLauncher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                airplaneLauncher.setPower(1.5);
            }

            if(gamepad1.dpad_up) {
                armBase.setPower(1.0);
            } else if(gamepad1.dpad_down) {
                armBase.setPower(-1.0);
            } else {
                armBase.setPower(0);
            }
            telemetry.addData("base position: ", "%7d", armBase.getCurrentPosition());

                double max;

                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;

                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                leftFrontDrive.setPower(0.6 * leftFrontPower);
                rightFrontDrive.setPower(0.6 * rightFrontPower);
                leftBackDrive.setPower(0.6 * leftBackPower);
                rightBackDrive.setPower(0.6 * rightBackPower);

                telemetry.update();
            }
        }
    }
