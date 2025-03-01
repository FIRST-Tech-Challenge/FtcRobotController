package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

// I couldn't remember how to push stuff into Git, so after I googled a bit I gave up.
// Git is weird and I don't like it. -paige
@Autonomous(name="Observation_Zone_Auto", group="Robot") // L1 stands for "Level 1 Hang"
public class Observation_Zone_Auto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null; //motor 0
    private DcMotor leftBackDrive = null; //motor 1
    private DcMotor rightFrontDrive = null; //motor 2
    private DcMotor rightBackDrive = null; //motor 3
    private DcMotor shoulder_left = null; //motor 0
    private DcMotor shoulder_right = null; //motor 1
    private DcMotor forearm = null; //motor 2

    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;

    static final double ARM_SPEED = 0.5;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        shoulder_left = hardwareMap.get(DcMotor.class, "shoulder_left");
        shoulder_right = hardwareMap.get(DcMotor.class, "shoulder_right");
        forearm = hardwareMap.get(DcMotor.class, "forearm");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        shoulder_left.setDirection(DcMotor.Direction.FORWARD);
        shoulder_right.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way



        //step 1: Strafe Right for 2.3 Seconds
        rightFrontDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(-FORWARD_SPEED);
        leftFrontDrive.setPower(-FORWARD_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }



        //step 2: Go to observation zone for 0.5 seconds
        rightFrontDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(FORWARD_SPEED);
        leftFrontDrive.setPower(FORWARD_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }



        // Step 3:  Stop
       shoulder_right.setPower(0);
        shoulder_left.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}