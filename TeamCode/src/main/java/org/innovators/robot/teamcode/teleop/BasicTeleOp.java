package org.innovators.robot.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.innovators.robot.teamcode.hardware.RobotHardware;
import org.innovators.robot.teamcode.util.Constants;

@TeleOp(name="Basic TeleOp", group="TeleOp")
public class BasicTeleOp extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Reset the runtime
        runtime.reset();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y * Constants.DRIVE_SPEED;
            double turn  =  gamepad1.right_stick_x * Constants.TURN_SPEED;
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Left Drive Power", leftPower);
            telemetry.addData("Right Drive Power", rightPower);
            telemetry.update();
        }
    }
}

