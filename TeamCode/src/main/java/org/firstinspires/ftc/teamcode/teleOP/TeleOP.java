package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "A GOATED TeleOP")
public class TeleOP extends LinearOpMode {
    Robot robot;
    GamepadEvents controller1, controller2;
    public static GamepadEvents.GamepadButton controlBinding1[];
    public static GamepadEvents.GamepadButton controlBinding2[];
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        controller1 = new GamepadEvents(gamepad1);
        controller2 = new GamepadEvents(gamepad2);

        controlBinding1 = new GamepadEvents.GamepadButton[]
                {
                        controller1.a,
                        controller1.b,
                        controller1.x,
                        controller1.right_bumper,
                        controller1.left_bumper

                };
        controlBinding2 = new GamepadEvents.GamepadButton[]
                {
                        controller2.dpad_up,
                        controller2.dpad_down,

                };
        waitForStart();
        robot.initializeStates();

        while(opModeIsActive())
        {
            robot.drive(-controller1.left_stick_y, controller1.left_stick_x, controller1.right_stick_x);
            robot.extend((controller1.left_trigger.getTriggerValue() - controller1.right_trigger.getTriggerValue())* 100);
            //a Button
            if(controlBinding1[0].onPress())
            {
                robot.intakeToggle();
            }
            if(controlBinding1[1].onPress())
            {
                robot.resetStates();
            }
            //x Button
            if(controlBinding1[2].onPress())
            {
                robot.toggleClaw();
            }
            //x Button && right bumper
            if(controlBinding1[2].onPress() && controlBinding1[3].onPress())
            {
                robot.ringClaw();
            }
            //Left Bumper. Commented out because hockey stick hasn't been tested out yet due to a
            // set screw being loose
//            if(controlBinding1[3].onPress())
//            {
//                robot.toggleHockeyStick();
//            }

            if(controlBinding2[0].onPress())
            {
                robot.adjustPivotOffset(1);
            }
            if(controlBinding2[1].onPress())
            {
                robot.adjustPivotOffset(-1);
            }

            robot.adjustHockeyOffset(-controller2.left_stick_y);


            controller1.update();
            controller2.update();

            robot.updatePos();

            telemetry.addLine("OPMODE ACTIVE");
            telemetry.update();
        }
    }
}
