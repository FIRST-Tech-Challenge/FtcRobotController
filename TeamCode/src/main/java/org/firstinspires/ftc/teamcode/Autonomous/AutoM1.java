package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.TeleOp.RobotHardware;

@Autonomous
public class AutoM1 extends LinearOpMode {

    public final RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        // Initialize robot hardware and AutoBase
        robot.init(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(robot, this);

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            // Use basePower method from AutoBase
            autoFunctions.basePower(0.5, 1000);
            autoFunctions.clawPower(0.5, 10000);

        }
    }
}