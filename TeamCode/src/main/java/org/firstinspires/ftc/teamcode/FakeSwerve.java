package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;



@TeleOp(name="FakeSwerve", group="Linear Opmode")

public class FakeSwerve extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    public double maxVelocity = 340 * 537.6; //for AndyMark NeveRest 20 motor when using .setVelocity()

    int direction = 0;

    boolean switchDriveMode = gamepad1.a;
    boolean driveMode = false;
    boolean hasSwitched = false;

    public void waitTime(double time){
        runtime.reset();
        while(runtime.seconds()<time){
        }
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

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
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double left  = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double right =  - gamepad1.right_stick_y;
            double y  = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotateX = gamepad1.right_stick_x;

            double leftFrontPower  = 0;
            double rightFrontPower = 0;
            double leftBackPower   = 0;
            double rightBackPower  = 0;

            //switches the mode drive train is using
            if(switchDriveMode){
                if(!hasSwitched){
                    driveMode = !driveMode;
                    hasSwitched = true;
                }
            }else{
                hasSwitched = false;
            }

            if(driveMode){ // pov mode (fake swerve)
                // Send calculated power to wheels
                max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotateX), 1);
                leftFrontDrive.setPower((y + x + rotateX) / max);
                leftBackDrive.setPower((y - x + rotateX) / max);
                rightFrontDrive.setPower((y - x - rotateX) / max);
                rightBackDrive.setPower((y + x - rotateX) / max);
            }else{ // tank drive
                if(left>0){
                    left *=left;
                }else{
                    left = left*left*-1;
                }

                if(right>0){
                    right *=right;
                }else{
                    right = right*right*-1;
                }

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                leftFrontPower  = left;
                rightFrontPower = right;
                leftBackPower   = left;
                rightBackPower  = right;

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));

                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                    leftBackPower   /= max;
                    rightBackPower  /= max;
                }

                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}