package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SciRavens-TeleOp")
public class RobotTeleop extends LinearOpMode {
    public Robot robot;
    public DriveTrain DT;
    public DroneLauncher DL;
    public Slider slider;
    public Arm arm;
    public Claw claw;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        DT = new DriveTrain(robot, gamepad1);
        DL = new DroneLauncher(robot, gamepad2);
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);
        claw = new Claw(robot, gamepad2);

        waitForStart();
        while(opModeIsActive()) {
            DT.drive();
            DL.launchDrone();
            slider.move();
            arm.move();
            claw.operate();
        }
    }
}
