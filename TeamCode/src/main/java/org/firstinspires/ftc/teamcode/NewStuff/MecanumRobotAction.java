package org.firstinspires.ftc.teamcode.NewStuff;

public class MecanumRobotAction extends Action {

    DriveTrain driveTrain;
    double targetTicks;
    double currentTicks;
    double error;
    double power;
    CalculateTickInches calculateTickInches = new CalculateTickInches();
    PIDController fLeftMecanumController;
    PIDController bRightMecanumController;

    double ERROR_TOLERANCE_IN_TICKS = 15;
    int counter = 0;

    public MecanumRobotAction(Action dependentAction, double inches, DriveTrain driveTrain) {
        this.dependentAction = dependentAction;
        this.targetTicks = calculateTickInches.inchToTicksDriveTrain(inches);
        this.driveTrain = driveTrain;
        fLeftMecanumController = new PIDController("fl mecanum", 0.001, 0.0000005, 0.4, true);
        bRightMecanumController = new PIDController("br mecanum", 0.001, 0.0000005, 0.4, true);
    }

    public MecanumRobotAction(double inches, DriveTrain driveTrain) {
        this.dependentAction = new DoneStateAction();
        this.targetTicks = calculateTickInches.inchToTicksDriveTrain(inches);
        this.driveTrain = driveTrain;
        fLeftMecanumController = new PIDController("fl mecanum", 0.005, 0.0000005, 0.4, true);
        bRightMecanumController = new PIDController("br mecanum", 0.005, 0.0000005, 0.4, true);
    }

    private void refreshError() {
        error = targetTicks - currentTicks;
    }

    @Override
    boolean checkDoneCondition() {
        refreshError();
        if(Math.abs(error) <= ERROR_TOLERANCE_IN_TICKS) {
            driveTrain.setPower(0, 0, 0, 0); // stop, to be safe
            driveTrain.getOpModeUtilities().getOpMode().sleep(100);
            return true;
        } else {
            return false;
        }
    }

    @Override
    void update() {

        this.currentTicks = driveTrain.getfLeftTicks();

        if (!hasStarted) {
            this.targetTicks += currentTicks;
            hasStarted = true;
        }

        if (counter < 3) {
            if ((Math.abs(fLeftMecanumController.state.lastError) < ERROR_TOLERANCE_IN_TICKS)) {
                counter++;
            } else { //todo: test this
                counter = 0;
            }

            currentTicks = driveTrain.getfLeftTicks();
            power = fLeftMecanumController.calculatePID(currentTicks, targetTicks);
            driveTrain.setPower(power, -1 * power, -1 * power, power);
        }
    }
}
