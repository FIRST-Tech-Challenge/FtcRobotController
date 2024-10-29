package org.innovators.robot.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.innovators.robot.teamcode.hardware.RobotHardware;
import org.innovators.robot.teamcode.util.Constants;

@Autonomous(name="Basic Autonomous", group="Autonomous")
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

    private void driveForward(double power, long time) {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);
        sleep(time);
        stopDriving();
    }

    private void driveBackward(double power, long time) {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(-power);
        sleep(time);
        stopDriving();
    }

    private void stopDriving() {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    private void pickUpBlock() {
        robot.armServo.setPosition(1.0); // Adjust the position as needed to pick up the block
        sleep(500); // Wait for the servo to move
    }

    private void dropBlock() {
        robot.armServo.setPosition(0.0); // Adjust the position as needed to drop the block
        sleep(500); // Wait for the servo to move
    }
}
