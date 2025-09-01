package org.firstinspires.ftc.team5898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Lift PID Control", group="Competition")
public class LiftPIDOpMode extends LinearOpMode {
    private RobotHardware robot;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        waitForStart();

        while (opModeIsActive()) {

            // Set a target encoder position for the lift (e.g., 500 ticks)
            robot.setLiftPosition(3000, 1);

            // Loop until both motors reach the target
            while (robot.slideLeft.isBusy() || robot.slideRight.isBusy()) {
                telemetry.addData("Left Position", robot.slideLeft.getCurrentPosition());
                telemetry.addData("Right Position", robot.slideRight.getCurrentPosition());
                telemetry.addData("Status", "Moving to position");
                telemetry.update();
            }

            telemetry.addData("Status", "Target reached");
            telemetry.addData("Left Position", robot.slideLeft.getCurrentPosition());
            telemetry.addData("Right Position", robot.slideRight.getCurrentPosition());
            telemetry.update();
            sleep(2000);



            /** manual control
            if (gamepad1.dpad_up) {
                robot.syncLift(0.8); // Raise lift with PID correction
            } else if (gamepad1.dpad_down) {
                robot.syncLift(-0.5); // Lower lift with PID correction
            } else {
                robot.syncLift(0); // Stop lift
            }**/
        }
    }
}
