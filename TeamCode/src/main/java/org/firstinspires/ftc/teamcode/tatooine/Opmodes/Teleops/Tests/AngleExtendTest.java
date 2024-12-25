package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

@TeleOp(name = "AngleExtendTest",group = "Tests")
public class AngleExtendTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor leftAngle = hardwareMap.get(DcMotor.class, "AL");
        DcMotor rightAngle = hardwareMap.get(DcMotor.class, "AR");
        DcMotor leftExtend = hardwareMap.get(DcMotor.class, "EL");
        DcMotor rightExtend = hardwareMap.get(DcMotor.class, "ER");
        leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftAngle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightAngle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();
        while (opModeIsActive()) {
            leftAngle.setPower(gamepad1.right_stick_y);
            rightAngle.setPower(-gamepad1.right_stick_y);
            leftExtend.setPower(gamepad1.left_stick_y);
            rightExtend.setPower(-gamepad1.left_stick_y);

            telemetry.addData("gamepad1.right_stick_y", -gamepad1.right_stick_y);
            telemetry.addData("gamepad1.left_stick_y", -gamepad1.left_stick_y);

            telemetry.addData("angleLeftTick", MathUtil.convertTicksToDegrees(28*70*(44/16), leftAngle.getCurrentPosition()));
            telemetry.addData("angleRightTick", MathUtil.convertTicksToDegrees(28*70*(44/16), rightAngle.getCurrentPosition()));
            telemetry.addData("extendLeftTick", MathUtil.convertTicksToDistance(537.7,3.8 , leftExtend.getCurrentPosition()));
            telemetry.addData("extendRightTick", MathUtil.convertTicksToDistance(537.7,3.8 , rightExtend.getCurrentPosition()));

            telemetry.update();
        }
    }
}
