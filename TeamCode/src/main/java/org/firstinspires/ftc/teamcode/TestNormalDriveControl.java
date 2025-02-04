package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "TestNormalDriveControl", group = "Test")
public class TestNormalDriveControl extends LinearOpMode {

    public DcMotor motorRightFront = null;
    public DcMotor motorRightBack = null;
    public DcMotor motorLeftBack = null;
    public DcMotor motorLeftFront = null;

    @Override
    public void runOpMode() {

        motorLeftBack = hardwareMap.dcMotor.get("backLeft");
        motorLeftFront = hardwareMap.dcMotor.get("frontLeft");
        motorRightFront = hardwareMap.dcMotor.get("frontRight");
        motorRightBack = hardwareMap.dcMotor.get("backRight");

        // Calling setPower with a positive value should rotate the wheel forward
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                directDriveControl(1.0);
            } else {
                directDriveControl(0.5);
            }

            telemetry.update();

            sleep(20);  // Sleeping here allows the CPU to catch up with other tasks
        }
    }

    private void directDriveControl(double speedMultiplier) {
        double max;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double r = gamepad1.right_stick_x;

        // Check out TeamCode/src/main/java/org/firstinspires/ftc/teamcode/docs/OmniWheelControlDerivation.md
        // for the derivation of these equations.
        double powerRightFront = -x + y - r;
        double powerRightBack  =  x + y - r;
        double powerLeftBack   = -x + y + r;
        double powerLeftFront  =  x + y + r;

        max = Math.max(Math.max(Math.abs(powerLeftFront), Math.abs(powerRightBack)),
                Math.max(Math.abs(powerLeftBack), Math.abs(powerLeftFront)));

        // If any individual motor power is greater than 1.0, scale all values to fit in the range [-1.0, 1.0]
        if (max > 1.0) {
            powerRightFront  /= max;
            powerRightBack   /= max;
            powerLeftBack    /= max;
            powerLeftFront   /= max;
        }

        motorRightFront.setPower(powerRightFront * speedMultiplier);
        motorRightBack.setPower(powerRightBack * speedMultiplier);
        motorLeftBack.setPower(powerLeftBack * speedMultiplier);
        motorLeftFront.setPower(powerLeftFront * speedMultiplier);
    }
}
