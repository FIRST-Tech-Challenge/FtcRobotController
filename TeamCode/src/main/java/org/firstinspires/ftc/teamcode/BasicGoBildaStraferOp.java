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
     * powerAdjust - Adjust the power based on the percent passed in.
     * @param power - Power value to adjust
     * @param percent - Percent to adjust power by
     * @return - Adjusted power value
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
        // Drive speed as a percent.
        // Set to 100 for full power.
        drivePercentage = 75;
        // Strafe speed as a percent.
        // Set to 100 for full power.
        strafePercentage = 75;
        // Rotate speed as a percent.
        // Set to 100 for full power.
        rotatePercentage = 45;

        waitForStart();
        if (opModeIsActive()) {
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
                moveRobot(driveValue, strafeValue, rotateValue);

                // Telemetry Data
                telemetry.addData("drive", driveValue);
                telemetry.addData("strafe", strafeValue);
                telemetry.addData("rotate", rotateValue);
                telemetry.update();
            }
        }
    }

    /**
     * moveRobot - Move the robot based on the power values passed in.
     * @param drive - Drive power value
     *              Positive values = forward
     *              Negative values = backward
     *              Range -1.0 to 1.0
     *              0 = no drive
     * @param strafe - Strafe power value
     *               Positive values = right
     *               Negative values = left
     *               Range -1.0 to 1.0
     *               0 = no strafe
     * @param rotate - Rotate power value
     *               Positive values = clockwise
     *               Negative values = counter clockwise
     *               Range -1.0 to 1.0
     *               0 = no rotate
     */
   private void moveRobot(double drive, double strafe, double rotate) {
        leftFrontDrive.setPower((drive + rotate) - strafe);
        rightFrontDrive.setPower((drive - rotate) + strafe);
        leftRearDrive.setPower(drive + rotate + strafe);
        rightRearDrive.setPower((drive - rotate) - strafe);
    }
}