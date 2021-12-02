/*
Made by Aryan Sinha,
FTC team 202101101
 */

package org.firstinspires.ftc.teamcode.Configs.oldConfig;

import static org.firstinspires.ftc.teamcode.Configs.oldConfig.selfDrive.AutoDriveUtils.logData;
import static org.firstinspires.ftc.teamcode.Configs.oldConfig.selfDrive.AutoDriveUtils.logLine;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This class handles the manual driving.
 * @author aryansinha
 */
@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class SimpleTeleOp extends BaseOpMode {
    private final Hardware2 robot = new Hardware2();

    /**
     * {@inheritDoc}
     */
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        logData(this, "Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        logLine(this, "The robot has been started");

        final ElapsedTime runtime = new ElapsedTime();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            /*
            What are these lines?
            Range.clip is a function where you set minimum and maximum values to "clip" a number
            Basically just meaning to keep the number in a certain range.
            Now why are we adding The drive and turn, and subtracting the drive and turn?
            This is a technique used by many teams. Basically think of this as "canceling out eachother"
            Lets assume our drive and turn values are both 0.5,
            so for the left wheel we'd get a power of 1,
            for the right wheel we'd get 0 power, So we'd be turning right.
            */
            double drive = -gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Send calculated power to wheels
            robot.getLeftDrive().setPower(leftPower);
            robot.getRightDrive().setPower(rightPower);

            if (gamepad1.left_bumper) {
                robot.getClaw().setPower(0.5);
                sleep(1);
                robot.getClaw().setPower(0);
            }
            else if (gamepad1.right_bumper) {
                robot.getClaw().setPower(-0.5);
                sleep(1);
                robot.getClaw().setPower(0);
            }

            else if (gamepad1.right_stick_button) {
                robot.getCarousel().setPosition(1);
            }
            else if (gamepad1.left_stick_button) {
                robot.getCarousel().setPosition(0);

            }

            if (gamepad1.right_trigger > 0) {
                robot.getClawServo().setPosition(1);
            }
            else if (gamepad1.left_trigger > 0) {
                robot.getClawServo().setPosition(0.1);
            }

            // Show the elapsed game time and wheel power.
            logData(this, "Status", "Run Time: " + runtime);
            logData(this, "Motors", String.format("left (%.2f), right (%.2f)", leftPower, rightPower));
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Hardware2 getRobot() {
        return robot;
    }
}