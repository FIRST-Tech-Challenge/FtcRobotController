package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Systems.Logic_Base;
import org.firstinspires.ftc.teamcode.Systems.RobotHardware;

class TeleOpExampleLogic extends Logic_Base { //You have to change the class name here

    public void execute_non_driver_controlled() {
        //this will have the telemetry, LEDs, etc.

        //Telemetry

        if (useRoadRunner) {
            robot.telemetry.addData("target x", target_x);
            robot.telemetry.addData("target y", target_y);

            robot.telemetry.addData("current x", current_x);
            robot.telemetry.addData("current y", current_y);
        }

        if ((useRoadRunner) || (usePID)) {
            robot.telemetry.addData("target angle", target_angle);
            robot.telemetry.addData("current angle", current_angle);

            robot.telemetry.addData("angle to field", angle());
        }

        //robot.telemetry.addData("Arm Position: ", robot.dc_motor_list[dc_motor_names.indexOf("arm")].getCurrentPosition());

        /* EXAMPLE - set LED color if distance sensor detects something
        if (robot.getDistInch("dSensor") < 4) robot.set_led_color("led", "Blue");
        else robot.set_led_color("led", "Green");
        EXAMPLE - set LED color if a is pressed
        if (buttons[keys.indexOf("operator a")]) robot.setLed("led", "Blue"); //note you have to subtract 20 if you want to access axis value
        else robot.setLed("led", "Green");
         */

        robot.telemetry.update();
        if (useRoadRunner) {
            position_tracker.update();
        }
    }

    //Initialization

    public void init() {
        setZeroAngle(-90); //Relative to Driver, Positive = Clockwise
                               //ex. if robot facing left, then starting angle = -90
        //target_positions[dc_motor_names.size() + servo_names.indexOf("right")] = 1.0;
        button_types[keys.indexOf("operator a")] = "button"; //toggle or default
    }

    public void init(StandardTrackingWheelLocalizer localizer) {
        init();
        initializeRoadRunner(45, 100, 90, localizer);
        //Direction Robot is facing; if facing left, then it is either -90° or 90°
    }

    public void set_keybinds() {

        //arm

        //new_keybind("arm", "operator right_stick_y", "default", 0.26, 0.13);
        //new_keybind("arm", "driver left_trigger", "cycle", 1, armPositions);

        //intake

        //new_keybind("intake", "driver a", "toggle", "normal", 0.3);
        //new_keybind("intake", "driver y", "toggle", "normal", -0.3);

        //duck

        //new_keybind("duckWheel", "driver b", "toggle", "gradient", 0.7);
        //new_keybind("duckWheel", "driver x", "toggle", "gradient", -0.7);

        //right

        //new_keybind("right", "driver dpad_up", "cycle", 1, servoPositions);

        //driver

        //new_keybind("goto", "driver right_bumper", 50, 80, "none");

    }

    /*
      KEY BINDS INSTRUCTIONS
        * We can have the same button control 2 different things, but it has to have the same type (button/cycle/toggle/default) each time
        * Button name has to be same as it appears in keys
        *
        * Default: active while held down
            * Default (axis): also has a multiplier of the axis depth for the power
        * Toggle: activate when pressed, then deactivate on next release
        * Button: only activate on moment that button is pushed down
        *
        * DC MOTORS
            * default (button) - 2 inputs; just set mode as "default", program will know if its a button or axis
                * mode ("gradient" or "normal" - gradient will always complete in 0.75 seconds)
                * power (double between -1.0, 1.0)
                    * if you put -1, 0, or 1 it will throw an error - has to be -1.0, 0.0, 1.0
            * default (axis) - 2 inputs
                * power if axis is negative (i.e. 0.26 if we want 0.26 power down - don't put -0.26)
                * power if axis is positive
                    * NOTE:
                        * for stick x: "negative" = trigger pushed to left
                        * for stick y: "negative" = trigger pushed down
                        * for triggers: put the value in the first input, leave the second one as 0.0
                * same requirements as power in default buttons
            * toggle - 2 inputs
                * inputs are identical as default (button)
            * button/cycle - 2 inputs
                * identical, except for cycles you can "cycle" back to the other end
                * how much we increase/decrease index on the list from where we are now
                * which list do we follow. Note, it must be a double[] array, even though dc motor positions are ints
        * SERVOS - same as DC motors except for the following:
            * for power, it's not servo power but instead half-revolutions/sec. (I think) Can be negative or positive; must be a double (i.e. not 2, -4).
            * only 1 input for default (button) or toggle: power. Cannot be a gradient so mode is useless. 2nd input can be anything.
        *
        * MOTION - buttons
            * input "zero" (where you normally put the motor/servo name) is instead "goto"
            * Can be either a button or axis
            * sets target x, target y as inputs 1/2
            * can also set target angle as input 3. Must be either a double or "none"
            * target x/y are with respect to the field, target angle is with respect to the player
        *
        * For any error, there will be an IllegalArgumentException thrown showing you where the error is.
      FEATURES
        * we can have multiple buttons, from either controller, access the same motor in
                different ways (i.e. they don't have to have the same parameters)
        * we can have the same button control 2 different motors
        * can change button functions freely - toggle, button, default and all their parameters
        * not all buttons have to be used
     */

    public TeleOpExampleLogic(RobotHardware r) { //You have to change this
        super(r);
        set_keybinds();
        set_button_types();
    }
}

@TeleOp(name="TeleOp Example", group="Iterative Opmode")
public class TeleOpExample extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    TeleOpExampleLogic logic = new TeleOpExampleLogic(robot); //You also have to change these two
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