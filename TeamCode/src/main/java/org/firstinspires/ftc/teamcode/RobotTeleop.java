package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SciRavens-TeleOp")
public class RobotTeleop extends LinearOpMode {
    public Robot robot;
    public DriveTrain DT;
    public DroneLauncher DL;
    public Slider slider;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;

    public Claw claw_wide;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    Leds leds;
private int cur = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        DT = new DriveTrain(robot, gamepad1);
        DL = new DroneLauncher(robot, gamepad1);
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);
        wrist = new Wrist(robot, gamepad2);
        claw = new Claw(robot.servoCR, robot.claw_left_close,robot.claw_left_wide_close, robot.claw_left_open, robot.servoCL, robot.claw_right_close,  robot.claw_right_wide_close, robot.claw_right_open);

        leds = new Leds(robot);
        leds.setPattern(0);

        waitForStart();
        leds.setPattern(cur);
        while(opModeIsActive()) {
            DT.drive();
            DL.launchDrone();
            slider.operate();
            arm_wrist_operate();
            claw_operate();
            leds_operate();
        }
    }

    private void arm_wrist_operate()
    {
        if (gamepad2.a) {
//            arm.setPosSample();
            wrist.setPosSample();
        } else if (gamepad2.b) {
            arm.setPosSample();
            wrist.setPosDrop();
        } else if(gamepad2.y) {
            arm.setPosFold();
            wrist.setPosBasket();
        } else if(gamepad2.x) {
            wrist.setPosSpecimen();
        }
    }

    private void claw_operate() {
        if (gamepad2.left_trigger > 0.9) {
            claw.close();
        } else if (gamepad2.right_trigger > 0.9) {
            claw.close_wide();
        } else {
            claw.open();
        }
    }
    private void leds_operate() {
        if (gamepad2.right_bumper || gamepad1.right_bumper) {
            cur = (cur + 1) % leds.patterns.length;
            leds.setPattern(cur);
            telemetry.addData("SETTING COLOR", leds.patterns[cur].toString());
            telemetry.update();
        }
    }
}

