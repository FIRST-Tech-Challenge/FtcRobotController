package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Grab Specimen AutoMode", group = "Autonomous")
public class AutoModeGrabSpecimen extends LumenBaseLinearOpMode {
    // Arm and wrist target positions
    private static final int ARM_POSITION_GRAB = 2000;  // Adjust as needed
    private static final int WRIST_POSITION_GRAB = 1500;  // Adjust as needed
    private static final double CLAW_OPEN_POSITION = 0.55;
    private static final double CLAW_CLOSED_POSITION = 0.7;

    @Override
    public void runOpMode() {
        // Initialize hardware components
        initHardware();

        int targetX = 0;
        int targetY = 20;
        // Set motor directions
        clawServo.setPosition(CLAW_OPEN_POSITION);  // Start with the claw open

        // Wait for the start button
        waitForStart();

        // Step 1: Drive to the specimen
        driveMecanum(targetY, targetX, 0);  // Move robot forward at 50% power (adjust as needed)
        sleep(500);  // Wait for the robot to move for 2 seconds

        // Step 2: Move arm to the correct position to grab the specimen
////        armMotor.setTargetPosition(ARM_POSITION_GRAB);
////        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        armMotor.setPower(0.5);  // Move arm at full power
////
////        // Step 3: Move wrist to the correct position
////        wristMotor.setTargetPosition(WRIST_POSITION_GRAB);
////        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        wristMotor.setPower(0.5);  // Move wrist at full power
//
//        // Wait for the arm and wrist to reach target positions
//        while (opModeIsActive() && (armMotor.isBusy() || wristMotor.isBusy())) {
//            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
//            telemetry.addData("Wrist Position", wristMotor.getCurrentPosition());
//            telemetry.update();
//        }

        // Step 4: Close the claw to grab the specimen
//        clawServo.setPosition(CLAW_CLOSED_POSITION);
//        sleep(500);  // Wait 500ms to ensure the claw closes
//
//        // Step 5: Retract arm and wrist after grabbing the specimen
//        armMotor.setTargetPosition(0);  // Move arm to initial position
//        wristMotor.setTargetPosition(0);  // Move wrist to initial position
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(0.5);  // Retract at full power
//        wristMotor.setPower(0.5);  // Retract at full power
//
//        // Wait for the arm and wrist to retract
//        while (opModeIsActive() && (armMotor.isBusy() || wristMotor.isBusy())) {
//            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
//            telemetry.addData("Wrist Position", wristMotor.getCurrentPosition());
//            telemetry.update();
//        }

        // Step 6: Drive robot to the home or parking position
        driveMecanum(-targetY, targetX, 0);  // Move robot backward (adjust as needed)
        sleep(500);  // Drive for 2 seconds

        stopAllMotors();  // Stop the robot after finishing the task
    }
}
