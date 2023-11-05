package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "ForwardDistanceOdometryAuto", preselectTeleOp = "IronEagle-GoBilda-Strafer")
public class ForwardDistanceOdometryAuto extends LinearOpMode {

    private DcMotor rightFrontDrive;
    private DcMotor rightRearDrive;
    private DcMotor leftRearDrive;
    private DcMotor leftFrontDrive;
    private DcMotor paraDeadWheelLeft;
    private DcMotor paraDeadWheelRight;
    private DcMotor perpDeadWheel;

    private int paraPositionLeft;
    private int paraPositionRight;
    private int perpPosition;
    private double current_forward_inches;
    private int target_forward_inches;

    double COUNTS_PER_INCH;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int COUNTS_PER_ENCODER_REV;
        int GEAR_REDUCTION;
        double WHEEL_CIRCUMFERENCE_INCHES;
        int COUNTS_PER_WHEEL_REV;
        double drivePower;

        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        paraDeadWheelLeft = hardwareMap.get(DcMotor.class, "paraDeadWheelLeft");
        paraDeadWheelRight = hardwareMap.get(DcMotor.class, "paraDeadWheelRight");
        perpDeadWheel = hardwareMap.get(DcMotor.class, "perpDeadWheel");

        // REV Robotics Through Bore Encoder specs
        COUNTS_PER_ENCODER_REV = 8192;
        // Dead Wheel to Encoder (no gears, so ratio is 1:1)
        GEAR_REDUCTION = 1;
        // Dual Omni 35mm (1.38 inches)
        WHEEL_CIRCUMFERENCE_INCHES = 1.38 * Math.PI;
        // Math to determine COUNTS_PER_INCH
        COUNTS_PER_WHEEL_REV = COUNTS_PER_ENCODER_REV * GEAR_REDUCTION;
        COUNTS_PER_INCH = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_INCHES;
        telemetry.addData("WHEEL_CIRCUMFERENCE_INCHES", WHEEL_CIRCUMFERENCE_INCHES);
        telemetry.addData("COUNTS_PER_WHEEL_REV", COUNTS_PER_WHEEL_REV);
        telemetry.addData("COUNTS_PER_INCH", COUNTS_PER_INCH);
        telemetry.update();

        // Set motor directions
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        // The 'Dead Wheels' will be the encoders we will use, so we disable the encoders on the drives themselves
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set 'Dead Wheels' to use encoder
        paraDeadWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        paraDeadWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Establish current and target position
        getCurrentPositions();
        target_forward_inches = 36;
        waitForStart();
        if (opModeIsActive()) {
            // Move forward until target position is reached
            drivePower = 0.25;
            rightFrontDrive.setPower(drivePower);
            rightRearDrive.setPower(drivePower);
            leftRearDrive.setPower(drivePower);
            leftFrontDrive.setPower(drivePower);

            while (current_forward_inches < target_forward_inches) {
                getCurrentPositions();
                addPositionTelemetryData();
                telemetry.update();
            }

            // Position reached, stop movement
            addPositionTelemetryData();
            telemetry.addLine("Position reached, stop movement");
            telemetry.update();

            drivePower = 0;
            rightFrontDrive.setPower(drivePower);
            rightRearDrive.setPower(drivePower);
            leftRearDrive.setPower(drivePower);
            leftFrontDrive.setPower(drivePower);
            // Sleep for the remainder of auto
            sleep(30000);
        }
    }

    private void getCurrentPositions() {
        paraPositionLeft = paraDeadWheelLeft.getCurrentPosition();
        paraPositionRight = paraDeadWheelRight.getCurrentPosition();
        perpPosition = perpDeadWheel.getCurrentPosition();
        current_forward_inches = paraPositionLeft / COUNTS_PER_INCH;
    }

    private void addPositionTelemetryData() {
        telemetry.addData("para position left", paraPositionLeft);
        telemetry.addData("para position right", paraPositionRight);
        telemetry.addData("perp position", perpPosition);
        telemetry.addData("current forward inches", current_forward_inches);
        telemetry.addData("target forward inches", target_forward_inches);
    }
}