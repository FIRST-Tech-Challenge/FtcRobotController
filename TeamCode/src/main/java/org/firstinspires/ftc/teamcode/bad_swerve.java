package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name="among us among us", group="sussy wussy")
public class bad_swerve extends LinearOpMode {

    private DcMotorEx motor1_right = null;
    private DcMotorEx motor2_right = null;
    private DcMotorEx motor1_left = null;
    private DcMotorEx motor2_left = null;

    @Override
    public void runOpMode() {
        motor1_right = hardwareMap.get(DcMotorEx.class, "r1");
        motor2_right = hardwareMap.get(DcMotorEx.class, "r2");

        motor1_left = hardwareMap.get(DcMotorEx.class, "l1");
        motor2_left = hardwareMap.get(DcMotorEx.class, "l2");

        motor1_right.setDirection(DcMotor.Direction.REVERSE);
        motor2_right.setDirection(DcMotor.Direction.FORWARD);

        motor1_left.setDirection(DcMotor.Direction.REVERSE);
        motor2_left.setDirection(DcMotor.Direction.FORWARD);

        motor1_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double left_y = -gamepad1.left_stick_y;
            double left_x = gamepad1.left_stick_x;
            double right_y = -gamepad1.right_stick_y;
            double right_x = gamepad1.right_stick_x;

            double[] left_angle_vector = {Math.atan2(left_x, left_y), Math.min(1, Math.hypot(left_x, left_y))};
            double[] right_angle_vector = {Math.atan2(right_x, right_y), Math.min(1, Math.hypot(right_x, right_y))};

            motor1_right.setPower(right_x);
            motor2_right.setPower(right_y);
            motor1_left.setPower(left_x); 
            motor2_left.setPower(left_y);

            telemetry.addData("Encoders", "motor 1: %d, motor 2: %d, difference: %d", motor1_right.getCurrentPosition(), motor2_right.getCurrentPosition(), Math.abs(motor1_right.getCurrentPosition()) - Math.abs(motor2_right.getCurrentPosition()));
            telemetry.addData("Current", "motor 1: %f, motor 2: %f", motor1_right.getCurrent(CurrentUnit.AMPS), motor2_right.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Angle Vector", "left angle: %f, left magnitude: %f\nright angle: %f, right magnitude: %f ", left_angle_vector[0], left_angle_vector[1], right_angle_vector[0], right_angle_vector[1]);

            telemetry.update();
        }
    }
}