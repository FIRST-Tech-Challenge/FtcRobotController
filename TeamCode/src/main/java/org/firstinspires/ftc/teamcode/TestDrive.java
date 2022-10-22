package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Test_Drive", group="Linear Opmode")

public class TestDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;   // Front Left
    private DcMotor fr = null;   // Front Right
    private DcMotor bl = null;   // Back Left
    private DcMotor br = null;   // Back Right

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fl  =    hardwareMap.get(DcMotor.class, "leftfront");
        fr =     hardwareMap.get(DcMotor.class, "rightfront");
        bl =     hardwareMap.get(DcMotor.class, "leftback");
        br =     hardwareMap.get(DcMotor.class, "rightback");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fl.setDirection(DcMotor.Direction.FORWARD);   //something odd with wiring.  todo
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);   //absolute infinity
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Starting at",  "%7d :%7d / %7d :%7d",
                fl.getCurrentPosition(), fr.getCurrentPosition(),
                bl.getCurrentPosition(), br.getCurrentPosition());
        telemetry.update();
        //lift.setDirection(DcMotor.Direction.REVERSE);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setTargetPosition(5);
        // Wait for the game to start (driver presses PLAY)
        //telemetry.addData("lift", lift.getCurrentPosition());
        //telemetry.addData("arm", arm.getPosition());
        //telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            fl.setPower(v1);
            fr.setPower(v2);
            bl.setPower(v3);
            br.setPower(v4);

            if (gamepad1.dpad_down){
                encoderDrive(DRIVE_SPEED,  -3,  -3, 2.0);
            }
            else if (gamepad1.dpad_up) {
                encoderDrive(DRIVE_SPEED,  3,  3, 2.0);
            }else if (gamepad1.dpad_right) {
                encoderDrive(DRIVE_SPEED,  3,  -3, 2.0);
            }else if (gamepad1.dpad_left) {
                encoderDrive(DRIVE_SPEED,  -3,  3, 2.0);
            }

        }
    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget, newLeftBackTarget;
        int newRightTarget, newRightBackTarget;

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = fl.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = fr.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = bl.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = br.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            fl.setTargetPosition(newLeftTarget);
            fr.setTargetPosition(newRightTarget);
            bl.setTargetPosition(newLeftBackTarget);
            br.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fl.setPower(Math.abs(speed));
            fr.setPower(Math.abs(speed));
            bl.setPower(Math.abs(speed));
            br.setPower(Math.abs(speed));
            // Display it for the driver.
            telemetry.addData("Starting at",  " %5d :%5d :%5d :%5d",
                    fl.getCurrentPosition(), fr.getCurrentPosition(),bl.getCurrentPosition(),br.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Running to",  " %5d :%5d :%5d :%5d", newLeftTarget,  newRightTarget,newLeftBackTarget,newRightBackTarget);
            telemetry.addLine();
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Running to",  " %5d :%5d :%5d :%5d", newLeftTarget,  newRightTarget,newLeftBackTarget,newRightBackTarget);
               // telemetry.addData("Currently at",  " %5d :%5d :%5d :%5d",
               //         fl.getCurrentPosition(), fr.getCurrentPosition(),bl.getCurrentPosition(),br.getCurrentPosition());

            }

            // Stop all motion;
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            telemetry.addData("Finished at",  " %5d :%5d :%5d :%5d",
                    fl.getCurrentPosition(), fr.getCurrentPosition(),bl.getCurrentPosition(),br.getCurrentPosition());
            telemetry.update();

            // Turn off RUN_TO_POSITION
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(5000);   // optional pause after each move.
        }
    }
}

