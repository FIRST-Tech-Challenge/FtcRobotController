package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Parking AutoMode", group = "Autonomous")
public class AutoModeParking extends LumenBaseLinearOpMode {

    @Override
    public void runOpMode() {
        initHardware();

        // Define movement parameters
        int forwardDistance = 12;   // Move forward just enough (12 inches) to grab the specimen
        int targetX = 0;   // No lateral movement, robot stays aligned with center
        int targetY = forwardDistance;  // Move forward just enough to reach the specimen

        clawServo.setPosition(CLAW_OPEN_POSITION);  // Start with the claw open

        // Wait for the start button
        waitForStart();

        // Step 1: Turn the robot to face the specimen
        driveMecanum(0, 0, 90);  // Turn 90 degrees (clockwise or counter-clockwise depending on your setup)
        sleep(500);  // Allow a brief pause for the robot to complete its turn

        // Step 2: Move forward a small distance (just to grab the specimen)
        driveMecanum(targetY, targetX, 0);  // Move forward by 12 inches (about 1 foot)
        sleep(1000);  // Allow 1 second for the robot to reach the specimen

        // Step 3: Move arm to the correct position to grab the specimen
        armMotor.setTargetPosition(800);  // Adjust arm position to grab the specimen (tune as needed)
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);  // Move arm at full power

        // Step 4: Move wrist to the correct position to grab the specimen
        wristMotor.setTargetPosition(180);  // Adjust wrist position as needed
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(1.0);  // Move wrist at full power

        // Wait for the arm and wrist to reach target positions
        while (opModeIsActive() && (armMotor.isBusy() || wristMotor.isBusy())) {
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Wrist Position", wristMotor.getCurrentPosition());
            telemetry.update();
        }

        // Step 5: Close the claw to grab the specimen
        clawServo.setPosition(CLAW_CLOSED_POSITION);
        sleep(500);  // Wait for the claw to close

        // Step 6: Retract arm and wrist after grabbing the specimen
        armMotor.setTargetPosition(0);  // Retract arm to its original position
        wristMotor.setTargetPosition(0);  // Retract wrist to its original position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);  // Retract at full power
        wristMotor.setPower(1.0);  // Retract at full power

        // Wait for the arm and wrist to retract
        while (opModeIsActive() && (armMotor.isBusy() || wristMotor.isBusy())) {
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Wrist Position", wristMotor.getCurrentPosition());
            telemetry.update();
        }

        // Step 7: Stop in the parking zone (robot is already there after the movements)
        stopAllMotors();  // Stop the robot after parking

        // Optional: You can also open the claw again after parking
        clawServo.setPosition(CLAW_OPEN_POSITION);

        telemetry.addData("Status", "Parking complete!");
        telemetry.update();
    }
}
