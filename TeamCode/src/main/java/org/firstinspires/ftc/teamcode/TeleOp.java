package org.firstinspires.ftc.teamcode;

@TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor linearSlide = null;
    private DcMotor Intake = null;
    private CRServo servo = null;
    private CRServo airplane = null;
    //private DcMotor linearSlide1;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        linearSlide = hardwareMap.get(DcMotor.class, "outputslide");
        Intake = hardwareMap.get(DcMotor.class, "intake");
        servo = hardwareMap.crservo.get("outputservo");
        airplane = hardwareMap.crservo.get("airplane");

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
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            boolean isReversed = false;
            double axial   = gamepad1.left_stick_y * 0.75;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * 0.75;
            double yaw     =  -gamepad1.right_stick_x * 0.65;
            double linearPower = 0.5;
            // }if(isReversed) {
            //     // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //     axial   = -axial;  // Note: pushing stick forward gives negative value
            //     lateral =  -lateral;
            //     yaw     =  -yaw;

            // }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            if(isReversed) {
                leftFrontPower = -leftFrontPower;
                leftBackPower = -leftBackPower;
                rightFrontPower = -rightFrontPower;
                rightBackPower = -rightBackPower;
            }

            //next task: make field oriented driving

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */
            // rightBack
            //leftBack
            //rightFront
            //leftFront

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            if(gamepad1.left_trigger > 0) {
                linearSlide.setPower(linearPower);
            } else if (gamepad1.right_trigger > 0) {
                linearSlide.setPower(-linearPower);
            } else {
                linearSlide.setPower(0.01);
            }

            Intake.setPower(-1);

            if (gamepad1.left_bumper) {
                servo.setPower(-0.4);
            } else if (gamepad1.right_bumper) {
                servo.setPower(0.4);
            }

            if(gamepad1.dpad_up) {
                airplane.setPower(-1);
            } else {
                airplane.setPower(0);
            }

            if(gamepad1.dpad_down) {
                isReversed = !isReversed;
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
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
    /*public void moveToPosition(double reference) {
        while (linearSlide.getCurrentPosition() != reference) {
            double power = PIDControl(reference, linearSlide.getCurrentPosition());
            linearSlide.setPower(power);
        }
    }*/
//}
}