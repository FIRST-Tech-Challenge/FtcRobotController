package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commons.RobotHardware;
import org.firstinspires.ftc.teamcode.teleOp.Claw;
import org.firstinspires.ftc.teamcode.teleOp.Hang;

@Autonomous (name = "AutoM1", preselectTeleOp = "ControlsNEW")
public class AutoM1 extends LinearOpMode {

    public final RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        // Initialize robot hardware and AutoBase
        robot.init(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(robot, this);
        Hang hangControls = new Hang(robot, this);
        Claw clawControls = new Claw(robot,this);
        clawControls.setClawServo(1, 0.3);

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            // Use basePower method from AutoBase
            clawControls.setClawPower(-1,500);
            clawControls.setClawRotation(0.40, 0.61 );
            sleep(2000);
            autoFunctions.shftPower(0.6,800);
            sleep(500);
            autoFunctions.basePower(0.6, 750);
            clawControls.setClawRotation(0.65, 0.34);
            sleep(500);
            clawControls.setClawServo(0.3,1);
            sleep(500);
            autoFunctions.basePower(-0.6, 800);
        }
    }
}