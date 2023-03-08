
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name="tommy")

public class Project1 extends LinearOpMode
{
    final static double ARM_HOME = 0.0;
    final static double ARM_MIN_RANGE = 0.0;
    final static double ARM_MAX_RANGE = 1.0;
    final static double HEXMOTOR_TPR = 288;
    final static double GOBILDA_TPR = 751.8; //5202
    final static double GEAR_REDUCTION = 1;
    final static double TICKS_PER_DEGREE = (HEXMOTOR_TPR * GEAR_REDUCTION) / 360;
    //ticks = pulses, cycles = ticks * 4

    final String[] debugModes =  {"VERT", "HORZ", "BUCKET", "BUCKET ANGLE", "CLAW", "CLAW ANGLE"};
    int dModesIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain = new MecanumDrive(robot);

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.horz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.vert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.imu.resetYaw();

        //variables
        double direction_y, direction_X, pivot, heading;
        int arm_target = 0, lift_target = 0;

        waitForStart();
        drivetrain.remote(0,0,0,0);
        while (opModeIsActive()) {
            direction_y   = gamepad1.left_stick_y;
            direction_X = -gamepad1.left_stick_x;
            pivot    =  gamepad1.right_stick_x;
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            drivetrain.remote(direction_y, direction_X, pivot, heading);

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) robot.imu.resetYaw();

            if (gamepad1.left_bumper) {
                robot.bucket.setPosition(1);
                robot.bucketAngle.setPosition(-0.4);
            }
            if (gamepad1.right_bumper) {
                robot.bucketAngle.setPosition(0.8);
                sleep(1500);
                robot.bucket.setPosition(0.8);
            }

            if (gamepad1.right_trigger > 0) robot.horz.setPower(-0.7 - (0.3 * gamepad1.right_trigger));
            else robot.horz.setPower(0.3);
            if (gamepad1.dpad_up) lift_target += 50;
            else if (gamepad1.dpad_down) lift_target -= 50;
            if (gamepad1.a) arm_target = 70;
            else if (gamepad1.b) arm_target = 0;

            Intake.arm(arm_target, robot, gamepad1);
            Intake.lift(lift_target, robot, gamepad1);


            telemetry.addData("Encoder value", robot.arm.getCurrentPosition());
            telemetry.addData("vert", robot.vert.getCurrentPosition());
            telemetry.addData("arm", robot.arm.getCurrentPosition());
            telemetry.update();

            //if (gamepad1.dpad_left) dModesIndex = (dModesIndex + 1) % debugModes.length;
            //else dModesIndex = (dModesIndex + 1) % debugModes.length;
            //Debug.control(debugModes[dModesIndex], robot, telemetry, gamepad1);
        }
    }
}