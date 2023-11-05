package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "BasicGoBildaStraferOp")
public class BasicGoBildaStraferOp extends LinearOpMode {

    private DcMotor rightFrontDrive;
    private DcMotor rightRearDrive;
    private DcMotor leftRearDrive;
    private DcMotor leftFrontDrive;

    /**
     * Adjust power based on power and percent value
     */
    private double powerAdjust(float power, double percent) {
        double result;

        result = (power * percent) / 100;
        return result;
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int drivePercentage;
        int strafePercentage;
        int rotatePercentage;
        double driveValue;
        double strafeValue;
        double rotateValue;

        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");

        // Set motor directions
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        // Drive speed power as a percent.
        // Set to 100 for full power.
        drivePercentage = 75;
        // Drive speed power as a percent.
        // Set to 100 for full power.
        strafePercentage = 75;
        // Drive speed power as a percent.
        // Set to 100 for full power.
        rotatePercentage = 45;

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Read Values
                driveValue = powerAdjust(-gamepad1.left_stick_y, drivePercentage);
                strafeValue = powerAdjust(-gamepad1.right_stick_x, strafePercentage);
                if (gamepad1.left_trigger > 0) {
                    rotateValue = powerAdjust(-1, rotatePercentage);
                } else if (gamepad1.right_trigger > 0) {
                    rotateValue = powerAdjust(1, rotatePercentage);
                } else {
                    rotateValue = powerAdjust(0, rotatePercentage);
                }
                // Execute Actions
                drivePower(driveValue, strafeValue, rotateValue);
                // Telemetry Data
                telemetry.addData("drive", driveValue);
                telemetry.addData("strafe", strafeValue);
                telemetry.addData("rotate", rotateValue);
                telemetry.update();
            }
        }
    }

    /**
     * Set motor drive power based on drive, strafe, and rotate inputs
     */
    private void drivePower(double drive, double strafe, double rotate) {
        leftFrontDrive.setPower((drive + rotate) - strafe);
        rightFrontDrive.setPower((drive - rotate) + strafe);
        leftRearDrive.setPower(drive + rotate + strafe);
        rightRearDrive.setPower((drive - rotate) - strafe);
    }
}