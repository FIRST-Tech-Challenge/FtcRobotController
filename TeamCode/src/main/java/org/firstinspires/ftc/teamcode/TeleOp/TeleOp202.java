package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Systems.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Systems.Logic_Base;
import org.firstinspires.ftc.teamcode.Systems.RobotHardware;

import java.util.Random;

class TeleOp202Logic extends Logic_Base {

    double time_difference = System.currentTimeMillis();
    double tx = 2.0;
    double ty = 0.0;

    double z1 = Math.PI / 180.0 * 5; //around 7.5 degrees idk
    double z2 = Math.PI / 180.0 * 354; //around 352 degrees idk

    double tpr = 2786.2109868741 / 2.0 / Math.PI;

    Random rand = new Random();
    DcMotor dc = null;

    public void execute_non_driver_controlled() {

        if (rand.nextDouble() > 1.2) throw new IllegalArgumentException("Sorry, that action is not allowed");

        double xbefor = tx;
        double ybefor = ty;

        time_difference = System.currentTimeMillis() - time_difference;
        if (buttons[keys.indexOf("driver dpad_up")])
            ty += time_difference * 0.002;
        if (buttons[keys.indexOf("driver dpad_down")])
            ty -= time_difference * 0.002;
        if (buttons[keys.indexOf("driver dpad_right")])
            tx += time_difference * 0.002;
        if (buttons[keys.indexOf("driver dpad_left")])
            tx -= time_difference * 0.002;

        time_difference = System.currentTimeMillis();

        if (ty < Math.sqrt(3)) {
            tx = 1;
            if (ty < -1) ty = -1;
        } else {
            tx = Math.sqrt(4 - ty * ty);
        }
        
        if (tx <= 0.001) tx = 0.001;

        if (buttons[keys.indexOf("driver y")]) {
            tx = 1;
            ty = 1.7;
        }

        double magnitude = Math.sqrt(tx * tx + ty * ty);

        if (magnitude > 2) {
            tx = xbefor;
            ty = ybefor;
            magnitude = Math.sqrt(tx * tx + ty * ty);
        }

        double tangle2 = Math.acos(1 - magnitude * magnitude / 2.0); //180 means straight line
        double tangle1 = Math.PI + Math.atan(ty / tx) - tangle2 / 2.0; //0 means straight down
        double tangle3 = Math.PI / 2.0 - tangle2 - tangle1;

        tangle1 -= z1;
        tangle2 -= z2;

        tangle1 *= tpr;
        tangle2 *= tpr;

        target_positions[0] = 0 - tangle1; //ticks per radian
        dc.setPower(robot.dc_motor_list[0].getPower());


        target_positions[1] = tangle2;

        robot.telemetry.addData("targetx", tx);
        robot.telemetry.addData("targety", ty);
        //target_positions[2] = tangle3 / 2.0 / Math.PI;
        robot.telemetry.addData("clawAligner", target_positions[2]);
        robot.telemetry.addData("target position", 1000 + 300 * rand.nextDouble());
        //set the servo to be the arm position

        robot.telemetry.update();
        if (useRoadRunner) {
            position_tracker.update();
        }
    }

    //Initialization

    public void init() {
        setZeroAngle(0);
        dc = robot.map.get(DcMotor.class, "joint1left");
        button_types[keys.indexOf("driver dpad_up")] = "default";
        button_types[keys.indexOf("driver dpad_down")] = "default";
        button_types[keys.indexOf("driver dpad_right")] = "default";
        button_types[keys.indexOf("driver dpad_left")] = "default";
    }

    public void init(StandardTrackingWheelLocalizer localizer) {
        init();
        initializeRoadRunner(45, 100, 90, localizer);
    }

    public void set_keybinds() {

        //new_keybind("claw", "driver a", "cycle", 1, new double[] {0.0, 1.0});
        new_keybind("clawAligner", "driver a", "default", 0.2, "it's great to be a michigan wolverine");
        new_keybind("clawAligner", "driver b", "default", -0.2, "it's great to be a michigan wolverine");

    }

    public TeleOp202Logic(RobotHardware r) {
        super(r);
        set_keybinds();
        set_button_types();
    }
}

@TeleOp(name="TeleOp Team 202", group="Iterative Opmode")
public class TeleOp202 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    TeleOp202Logic logic = new TeleOp202Logic(robot);
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