package ftc_library.manipulator;

/**
 * ManipulatorCombinations provides methods for complex manipulator actions,
 * such as moving the arm and claw simultaneously.
 */
public class ManipulatorCombinations {
    private final Arm arm;
    private final Claw claw;

    public ManipulatorCombinations(Arm arm, Claw claw) {
        this.arm = arm;
        this.claw = claw;
    }

    /**
     * Lifts the arm while opening the claw.
     * @param armPower Power for vertical arm movement
     */
    public void liftAndOpenClaw(double armPower) {
        arm.moveVertical(armPower);
        claw.open();
    }

    /**
     * Extends the arm horizontally while closing the claw.
     * @param armPower Power for horizontal arm movement
     */
    public void extendAndCloseClaw(double armPower) {
        arm.moveHorizontal(armPower);
        claw.close();
    }

    /**
     * Moves both vertical and horizontal arms at the same time.
     * @param verticalPower   Power for vertical arm
     * @param horizontalPower Power for horizontal arm
     */
    public void moveArmDiagonal(double verticalPower, double horizontalPower) {
        arm.moveVertical(verticalPower);
        arm.moveHorizontal(horizontalPower);
    }
}
