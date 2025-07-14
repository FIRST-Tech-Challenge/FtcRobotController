package ftc_library.drive;

/**
 * DriveCombinations provides advanced movement options for a mecanum drive.
 * Useful for diagonal moves or combining strafing and rotation.
 */
public class DriveCombinations {
    private final MecanumDrive mecanumDrive;

    public DriveCombinations(MecanumDrive mecanumDrive) {
        this.mecanumDrive = mecanumDrive;
    }

    // Diagonal movements
    public void driveDiagonalFrontLeft(double power) { mecanumDrive.drive(power, -power, 0); }
    public void driveDiagonalFrontRight(double power) { mecanumDrive.drive(power, power, 0); }
    public void driveDiagonalBackLeft(double power) { mecanumDrive.drive(-power, -power, 0); }
    public void driveDiagonalBackRight(double power) { mecanumDrive.drive(-power, power, 0); }

    // Move while rotating
    public void driveForwardAndRotate(double power, double rotate) { mecanumDrive.drive(power, 0, rotate); }
    public void strafeAndRotate(double power, double rotate) { mecanumDrive.drive(0, power, rotate); }
}
