package ftc_library;

import ftc_library.drive.FieldOrientedDrive;
import ftc_library.manipulator.Arm;
import ftc_library.manipulator.Claw;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * TeleOpController handles all driver and operator controls during TeleOp.
 * This includes field-oriented driving and manipulator controls.
 */
public class TeleOpController {
    private final FieldOrientedDrive fodDrive;
    private final Arm arm;
    private final Claw claw;
    private boolean emergencyDrive = false;
    private boolean previousX = false;

    public TeleOpController(FieldOrientedDrive fodDrive, Arm arm, Claw claw) {
        this.fodDrive = fodDrive;
        this.arm = arm;
        this.claw = claw;
    }

    /**
     * Processes all gamepad inputs for driving and manipulators.
     * @param gamepad1 Driver gamepad (movement)
     * @param gamepad2 Operator gamepad (arm/claw)
     */
    public void handleGamepadInputs(Gamepad gamepad1, Gamepad gamepad2) {
        // Toggle between field-oriented and emergency drive with X button
        boolean x = gamepad1.x;
        if (!x && previousX) {
            emergencyDrive = !emergencyDrive;
        }
        previousX = x;

        boolean slowMode = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean leftBumper = gamepad1.left_bumper;

        // Choose driving mode
        if (emergencyDrive) {
            fodDrive.emergencyDrive(gamepad1, rightBumper, leftBumper);
        } else {
            fodDrive.drive(gamepad1, slowMode, rightBumper);
        }

        // Arm and Claw controls (customize as needed)
        arm.moveVertical(gamepad2.right_stick_y);
        arm.moveHorizontal(gamepad2.left_stick_y);
        if (gamepad2.a) claw.open();
        if (gamepad2.b) claw.close();
        if (gamepad2.x) claw.halfOpen();
    }
}
