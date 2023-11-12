package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="JustDrivetrain")
public class Drivetrain extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;
    //private DcMotor linearSlide1;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        //linearSlide1 = hardwareMap.get(DcMotor.class, "linear_slide_1");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        boolean isReversed = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            double axial   = -gamepad1.left_stick_y * 0.75;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * 0.75;
            double yaw     =  gamepad1.right_stick_x * 0.65;

            if(isReversed) {
                leftFrontPower = -(axial + lateral + yaw);
                leftBackPower = -(axial - lateral - yaw);
                rightFrontPower = -(axial - lateral + yaw);
                rightBackPower = -(axial + lateral - yaw);
            }else {
                leftFrontPower  = axial + lateral + yaw;
                rightFrontPower = axial - lateral - yaw;
                leftBackPower   = axial - lateral + yaw;
                rightBackPower  = axial + lateral - yaw;
            }

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            if (gamepad1.a) {
                isReversed = !isReversed;
                telemetry.addData("reversed!", isReversed);
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Front left wheel", leftFrontDrive.getCurrentPosition());

            telemetry.addData("Front Left Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("Front Right Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Back Left Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("Back Right Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
    }

}
