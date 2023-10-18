package org.firstinspires.ftc.team8923_CENTERSTAGE;


abstract public class BaseTeleOp extends BaseOpMode {

    static final double INTAKE_SPEED = 0.7;
    static final double SLIDES_SPEED = 0.6;

    double driveSpeed = 0.8;
    double mechanismSpeed = 0.9;

    /*public void driveRobot() { This is not called
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        double angle = Math.toDegrees(Math.atan2(y, x));
        double power = calculateDistance(x,y);

        driveMecanum(x, y, pivot);
    }*/

    public void teleopDriving() {
        // driveMechanism(INTAKE_SPEED, SLIDES_SPEED);
       // driveRobot();
    }

    /*public void driveMechanism(double speedInput, double speedOutput) {
        leftIntake.setPosition();
        // yippee combine all of those methods
    }

    public void driveIntake(boolean direction) {
        // spin compliant wheels using the two motors and use constant speed

    }
    public void transferFromIntakeToOutput(double speed, boolean direction) {
        // tbh idk what this means
    }
    public void driveOutput(double speed, double uhhhhhhhhhh) {

    } */
}
