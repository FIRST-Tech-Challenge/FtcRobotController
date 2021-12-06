package org.firstinspires.ftc.teamcode.robots.reach.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Jank Trike OpMode", group="Linear Opmode")
public class JankTrikeOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor swerve = null;
    private DcMotor swerveAngle = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        swerve  = hardwareMap.get(DcMotor.class, "swerve");
        swerveAngle = hardwareMap.get(DcMotor.class, "swerveAngle");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        swerve.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        swerveAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swerveAngle.setTargetPosition(swerveAngle.getCurrentPosition());
        swerveAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        swerveAngle.setPower(1);
        swerveAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    double swerveTarget = 0;

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double swervePower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;

        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        double driveSwerve = gamepad2.left_stick_y;
        double turnSwerve  = 2 * gamepad2.right_stick_x;

        swerveTarget += turnSwerve;

        swervePower    = Range.clip(driveSwerve, -1.0, 1.0) ;
        swerveTarget   = Range.clip(swerveTarget, -443.0, 443.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        swerve.setPower(swervePower);
        swerveAngle.setTargetPosition((int)(swerveTarget));

        // Show the elapsed game time and wheel power.\q
        telemetry.addData("Motors", leftPower);
        telemetry.addData("ticksLeft", leftDrive.getCurrentPosition());
        telemetry.addData("ticksLeft", rightDrive.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

