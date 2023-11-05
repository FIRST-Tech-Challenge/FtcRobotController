package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Basic Tank Control")
public class BasicTankControlOp extends LinearOpMode {
    private DcMotor motorRF;
    private DcMotor motorLB;
    private DcMotor motorLF;
    private DcMotor motorRB;

    int drivePercentage = 150;

    enum TankSide {
        LEFT,
        RIGHT
    }

    @Override
    public void runOpMode() {
        motorRF = hardwareMap.get(DcMotor.class, "motorRF");
        motorLB = hardwareMap.get(DcMotor.class, "motorLB");
        motorLF = hardwareMap.get(DcMotor.class, "motorLF");
        motorRB = hardwareMap.get(DcMotor.class, "motorRB");

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Basic Drive of the Robot
            setDrivePower(TankSide.LEFT, gamepad1.left_stick_y);
            setDrivePower(TankSide.RIGHT, gamepad1.right_stick_y);

            telemetry.addData("Left Pow", motorLF.getPower());
            telemetry.addData("Right Pow", motorRB.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    /**
     * Adjust the value of power by the value of percent
     * This helps customize the speed of motors when using thumbstick controls
     */
    private double powerAdjust(double power, double percent) {
        double result;

        result = (power * percent) / 100;
        return result;
    }

    /**
     * Set both motors on a tank side to power
     * @param side tank side
     * @param power motor power
     */
    private void setDrivePower(TankSide side, float power) {
        double adjPower = powerAdjust(power, drivePercentage);
        if (TankSide.LEFT == side) {
            motorLF.setPower(adjPower);
            motorLB.setPower(adjPower);
        } else {
            motorRF.setPower(adjPower);
            motorRB.setPower(adjPower);
        }
    }
}
