package org.firstinspires.ftc.teamcode;

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

        waitForStart();

        PIDController controller = new PIDController(0.003, 0.0000001, 0.4);
        double currentPos = robot.fLeft.getCurrentPosition();
        double targetPos = currentPos + controller.convertInchesToTicks(12);
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
    }
}
