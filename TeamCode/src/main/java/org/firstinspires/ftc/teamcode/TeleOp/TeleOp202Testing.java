package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Systems.Logic_Base;
import org.firstinspires.ftc.teamcode.Systems.RobotHardware;

import java.util.Random;

class TeleOp202TestingLogic extends Logic_Base {

    public void execute_non_driver_controlled() {

        robot.telemetry.addData("joint1left data", robot.dc_motor_list[0].getCurrentPosition());
        robot.telemetry.addData("joint1left data", target_positions[0]);

        robot.telemetry.addData("joint1right data", robot.dc_motor_list[1].getCurrentPosition());
        robot.telemetry.addData("joint1right data", target_positions[1]);

        robot.telemetry.addData("joint2 data", robot.dc_motor_list[2].getCurrentPosition());
        robot.telemetry.addData("joint2 data", target_positions[2]);
        robot.telemetry.update();
        if (useRoadRunner) {
            position_tracker.update();
        }
    }

    //Initialization

    public void init() {
        setZeroAngle(0);
    }

    public void init(StandardTrackingWheelLocalizer localizer) {
        init();
        initializeRoadRunner(45, 100, 90, localizer);
    }

    public void set_keybinds() {

        new_keybind("joint1left", "operator left_stick_y", "default", 0.2, 0.2);

        new_keybind("joint1right", "operator left_stick_y", "default", 0.2, 0.2);

        new_keybind("joint2", "operator right_stick_y", "default", 0.2, 0.2);

        new_keybind("claw", "operator a", "default", 0.2, "it's great to be a michigan wolverine");
        new_keybind("claw", "operator b", "default", -0.2, "it's great to be a michigan wolverine");

    }

    public TeleOp202TestingLogic(RobotHardware r) {
        super(r);
        set_keybinds();
        set_button_types();
    }
}

@TeleOp(name="TeleOp Team 202 Testing", group="Iterative Opmode")
public class TeleOp202Testing extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    TeleOp202TestingLogic logic = new TeleOp202TestingLogic(robot);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();
        if (logic.useRoadRunner) {
            StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap);
            logic.init(localizer);
        } else {
            logic.init();
        }
        while (opModeIsActive()) {
            logic.execute_controllers(gamepad1, gamepad2); //driver is gamepad1, operator is gamepad2
            logic.execute_non_driver_controlled();
        }
    }
}