package org.firstinspires.ftc.teamcode.actions;

import android.util.Log;

import org.firstinspires.ftc.teamcode.math.CalculateTickInches;
import org.firstinspires.ftc.teamcode.PID.PIDController2023;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;

public class MecanumRobotAction extends Action {

    DriveTrain driveTrain;
    double targetTicks;
    double currentTicks;
    double error;
    double power;
    CalculateTickInches calculateTickInches = new CalculateTickInches();
    PIDController2023 fLeftMecanumController;
    PIDController2023 bRightMecanumController;

    double ERROR_TOLERANCE_IN_TICKS = 10;
    int counter = 0;

    public MecanumRobotAction(double inches, DriveTrain driveTrain) {
        this.dependentAction = new DoneStateAction();
        this.driveTrain = driveTrain;
        fLeftMecanumController = new PIDController2023("fl mecanum", 0.005, 0.0000005, 0.4, true);
        bRightMecanumController = new PIDController2023("br mecanum", 0.005, 0.0000005, 0.4, true);
        this.targetTicks = fLeftMecanumController.convertInchesToTicks(inches);
    }

    private void refreshError() {
        error = targetTicks - currentTicks;
        Log.d("mecanum", "error is " + error);
    }

    @Override
    public boolean checkDoneCondition() {
        refreshError();
        if(Math.abs(error) <= ERROR_TOLERANCE_IN_TICKS) {
            driveTrain.setPower(0); // stop, to be safe
            Log.d("mecanum", "done");
            driveTrain.getOpModeUtilities().getOpMode().sleep(100);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {

        this.currentTicks = driveTrain.getfLeftTicks();

        if (!hasStarted) {
            fLeftMecanumController.state.integral = 0;
            fLeftMecanumController.state.lastError = 0;
            fLeftMecanumController.state.lastTime = 0;

            this.targetTicks += currentTicks;
            Log.d("mecanum", "target ticks " + targetTicks);
            hasStarted = true;
        }

        if (counter < 3) {
            if ((Math.abs(fLeftMecanumController.state.lastError) < ERROR_TOLERANCE_IN_TICKS)) {
                counter++;
            } else { //todo: test this
                counter = 0;
            }

            currentTicks = driveTrain.getfLeftTicks();
            Log.d("mecanum", "current ticks is " + currentTicks);
            power = fLeftMecanumController.calculatePID(currentTicks, targetTicks);
            Log.d("mecanum", "power is " + power);

            if(!isDone){
                driveTrain.setPower(power, -1 * power, -1 * power, power);
            }
        }
    }
}
