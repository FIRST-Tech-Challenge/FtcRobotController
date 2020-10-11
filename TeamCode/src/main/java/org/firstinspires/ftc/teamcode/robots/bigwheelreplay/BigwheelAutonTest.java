package org.firstinspires.ftc.teamcode.robots.bigwheelreplay;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.robots.bigwheelreplay.BigwheelAutonRecorder.GamepadPair;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;

import static org.firstinspires.ftc.teamcode.robots.bigwheelreplay.BigwheelAutonData.log;

@Autonomous(name="Big Wheel Auton Test", group="Linear Opmode")
public class BigwheelAutonTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor rightDrive2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive1  = hardwareMap.get(DcMotor.class, "left_drive_1");
        leftDrive2  = hardwareMap.get(DcMotor.class, "left_drive_2");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive_1");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive_2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive1.setDirection(REVERSE);
        leftDrive2.setDirection(REVERSE);
        rightDrive1.setDirection(FORWARD);
        rightDrive2.setDirection(FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        for (int i = 0; opModeIsActive() && i < log.size(); i++) {
            GamepadPair pair = log.get(i);
            runIteration(pair);
            sleep(100);
        }
    }

    public void runIteration(GamepadPair pair) {
        double drive = -pair.y;
        double turn  =  pair.x;

        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        leftDrive1.setPower(leftPower);
        leftDrive2.setPower(leftPower);
        rightDrive1.setPower(rightPower);
        rightDrive2.setPower(rightPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
    }
}
