package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Systems.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Systems.Logic_Base;
import org.firstinspires.ftc.teamcode.Systems.RobotHardware;

class TeleOp201Logic extends Logic_Base {

    public void execute_non_driver_controlled() {

        if (buttons[keys.indexOf("driver a")]) {
            robot.telemetry.addData("a button", "pressed (should show up for only one tick)");
            //set scissor position to closed
            //if V4B greater than position 1
                //if arm not close to position 1: move arm
                //else: move V4B, variable + wait for 1 sec or smth
            //else if arm not close to position 2: move arm to position 2
            //else: V4B to final position
            //finally, variable for a cycle is turned off

            //IF all of these checks are passed on the first step, then a cycle 2 is true
            //move arm down
            //move servo to open, wait a few seconds
            //move arm up
            //move v4b inward
            //move arm up again
            //move v4b out again
            //make this second stage on a second button to start but hopefully it can all be on one by the end :)
        }
        robot.telemetry.addData("Left DCM", robot.dc_motor_list[0].getCurrentPosition());
        robot.telemetry.addData("Right DCM", robot.dc_motor_list[1].getCurrentPosition());

        robot.telemetry.addData("Left DCM Target", target_positions[0]);
        robot.telemetry.addData("Right DCM Target", target_positions[1]);

        robot.telemetry.addData("Virtual", robot.servo_list[0].getPosition());
        robot.telemetry.addData("Scissor", robot.servo_list[1].getPosition());
        robot.telemetry.addData("Angle?", robot.getAngle());
        robot.telemetry.addData("Angle V2", current_angle = 0 - robot.getAngle() - zero_angle); //Only different value if not starting robot straight ahead
                                    //Positive = Rotated clockwise
        robot.telemetry.update();

        target_positions[1] = 0 - target_positions[0];

        if (useRoadRunner) {
            position_tracker.update();
        }
    }

    //Initialization

    public void init() {
        setZeroAngle(0);
        button_types[keys.indexOf("driver a")] = "button";
    }

    public void init(StandardTrackingWheelLocalizer localizer) {
        init();
        initializeRoadRunner(45, 100, 90, localizer);
    }

    public void set_keybinds() {

        // Arm
        new_keybind("Left", "driver dpad_up", "default", "gradient", 0.8);
        new_keybind("Right", "driver dpad_up", "default", "gradient", 0.8);

        new_keybind("Left", "driver dpad_down", "default", "gradient", -0.3);
        new_keybind("Right", "driver dpad_down", "default", "gradient", -0.3);

        // V4B
        new_keybind("Virtual", "driver y", "default", "gradient", 0.3);
        new_keybind("Virtual", "driver x", "default", "gradient", -0.3);

    }

    public TeleOp201Logic(RobotHardware r) {
        super(r);
        set_keybinds();
        set_button_types();
    }
}

@TeleOp(name="TeleOp", group="Iterative Opmode")
public class TeleOp201 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    TeleOp201Logic logic = new TeleOp201Logic(robot);
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