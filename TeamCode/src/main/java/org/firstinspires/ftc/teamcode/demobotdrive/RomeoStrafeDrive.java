package org.firstinspires.ftc.teamcode.demobotdrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Romeo strafe drive", group="Linear OpMode")
public class RomeoStrafeDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private double slowSpeed = 0.4;
    private double fullSpeed = 1;
    private double currentSpeed = 1;
    private boolean canChangeSpeeds = true;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double forward   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double strafe =  gamepad1.left_stick_x;
            double turn     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = forward + strafe + turn;
            double rightFrontPower = forward - strafe - turn;
            double leftBackPower   = forward - strafe + turn;
            double rightBackPower  = forward + strafe - turn;

            if (gamepad1.y && canChangeSpeeds) {
                canChangeSpeeds = false;
                if (currentSpeed == fullSpeed) {
                    currentSpeed = slowSpeed;
                } else {
                    currentSpeed = fullSpeed;
                }
            } else if (!gamepad1.y) {
                canChangeSpeeds = true;
            }


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * currentSpeed);
            rightFrontDrive.setPower(rightFrontPower * currentSpeed);
            leftBackDrive.setPower(leftBackPower * currentSpeed);
            rightBackDrive.setPower(rightBackPower * currentSpeed);
        }
    }}
