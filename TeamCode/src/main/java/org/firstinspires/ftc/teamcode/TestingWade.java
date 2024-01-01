package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class TestingWade extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        int distance = 10;

        waitForStart();

        PIDController controller = new PIDController(0.003, 0.0000005, 0.4);
        double currentPos = robot.fLeft.getCurrentPosition();
        double targetPos = currentPos + controller.convertInchesToTicks(distance);
        double power;
        double ERROR_TOLERANCE_IN_TICKS = 15;
        int counter = 0;

        while (opModeIsActive() && counter < 3) {
            if (Math.abs(controller.lastError) < ERROR_TOLERANCE_IN_TICKS) {
                counter++;
            }

            currentPos = robot.fLeft.getCurrentPosition();
            power = controller.calculatePID(currentPos, targetPos);
            // make sure there is enough power - unless integral takes care of it
            // make sure there isn't too much power
            robot.setMotorPower(power, power, power, power);
        }

        robot.setMotorPower(0, 0, 0, 0); // stop, to be safe
        this.sleep(500);
        double distanceTraveled = controller.convertTicksToInches(targetPos - robot.fLeft.getCurrentPosition());
        Log.d("new pid", "runOpMode: final error in inches " + distanceTraveled);
    }
}
