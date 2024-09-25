package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Time", group="Robot")
public class Time extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware. Prefix any hardware function with "robot." to
    // access this class.
    RobotHardware robot = new RobotHardware(this);


    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();


        waitForStart();
        robot.runtime.reset();
        // Step  through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // Step 1: Strafe to the right for 1 second.
        robot.driveRobot(0, RobotHardware.FORWARD_SPEED,0);
        robot.runtime.reset();
        while (opModeIsActive() && (robot.runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", robot.runtime.seconds());
            telemetry.update();
        }
    }
}
