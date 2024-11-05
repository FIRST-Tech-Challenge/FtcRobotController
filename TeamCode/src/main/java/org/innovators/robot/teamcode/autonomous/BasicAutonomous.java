package org.innovators.robot.teamcode.autonomous;
import static org.innovators.robot.teamcode.util.Constants.SERVO_POWER_DOWN;
import static org.innovators.robot.teamcode.util.Constants.SERVO_POWER_UP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.innovators.robot.teamcode.hardware.RobotHardware;
import org.innovators.robot.teamcode.util.Constants;

@Autonomous(name="Autonomous", group="Autonomous")
public class BasicAutonomous extends LinearOpMode {
    protected RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step 1: Drive forward to the block
        driveForward(Constants.DRIVE_SPEED, 1000); // Drive forward at 50% power for 1000 milliseconds

        // Step 2: Pick up the block
        pickUpBlock();

        // Step 3: Drive to the basket
        driveForward(Constants.DRIVE_SPEED, 1000); // Adjust the distance as needed

        // Step 4: Drop the block in the basket
        dropBlock();

        // Step 5: Back away from the basket
        driveBackward(Constants.DRIVE_SPEED, 500); // Drive backward at 50% power for 500 milliseconds
    }

    protected void driveForward(double power, long time) {
        robot.leftDriveMotor.setPower(power);
        robot.rightDriveMotor.setPower(power);
        sleep(time);
        stopDriving();
    }

    protected void driveBackward(double power, long time) {
        robot.leftDriveMotor.setPower(-power);
        robot.rightDriveMotor.setPower(-power);
        sleep(time);
        stopDriving();
    }

    private void stopDriving() {
        robot.leftDriveMotor.setPower(0);
        robot.rightDriveMotor.setPower(0);
    }

    protected void pickUpBlock() {
        robot.armWristTorqueServo.setPosition(SERVO_POWER_UP); // Adjust the position as needed to pick up the block
        sleep(500); // Wait for the servo to move
    }

    protected void dropBlock() {
        robot.armWristTorqueServo.setPosition(SERVO_POWER_DOWN); // Adjust the position as needed to drop the block
        sleep(500); // Wait for the servo to move
    }
}
