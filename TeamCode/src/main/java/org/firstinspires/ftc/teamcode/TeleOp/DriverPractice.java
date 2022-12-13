package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Systems.Logic_Base;
import org.firstinspires.ftc.teamcode.Systems.RobotHardware;

class DriverPracticeLogic extends Logic_Base {

    public void execute_non_driver_controlled() {

        if (useRoadRunner) {
            robot.telemetry.addData("target x", target_x);
            robot.telemetry.addData("target y", target_y);

            robot.telemetry.addData("current x", current_x);
            robot.telemetry.addData("current y", current_y);
        }

        if ((useRoadRunner) || (usePID)) {
            robot.telemetry.addData("target angle", target_angle);
            robot.telemetry.addData("current angle", current_angle);
        }

        robot.telemetry.addData("angle to field", angle());

        if (buttons[keys.indexOf("driver a")]) {
            robot.telemetry.addData("resetting", "imu angle");
            resetZeroAngle();
        }

        robot.telemetry.update();
        if (useRoadRunner) {
            position_tracker.update();
        }
    }


    public void init() {
        setZeroAngle(0);
        button_types[keys.indexOf("driver a")] = "button";
    }

    public void init(StandardTrackingWheelLocalizer localizer) {
        init();
        initializeRoadRunner(45, 100, 90, localizer);
    }

    public void set_keybinds() {

    }

    public DriverPracticeLogic(RobotHardware r) {
        super(r);
        set_keybinds();
        set_button_types();
    }
}

@TeleOp(name="Driver Practice", group="Iterative Opmode")
public class DriverPractice extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    DriverPracticeLogic logic = new DriverPracticeLogic(robot);
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