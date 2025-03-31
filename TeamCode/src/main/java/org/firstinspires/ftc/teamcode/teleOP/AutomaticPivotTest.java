package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AutomaticPivot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="Automatic Pivot Test")
public class AutomaticPivotTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEvents controller1 = new GamepadEvents(gamepad1);
        DriveTrain robot = new DriveTrain(hardwareMap,"frontLeft", "backLeft", "frontRight",
                "backRight");

        AutomaticPivot pivot = new AutomaticPivot(hardwareMap);

        waitForStart();
        pivot.init();

        while(opModeIsActive()){
//            robot.drive(-controller1.left_stick_y ,controller1.left_stick_x, controller1.right_stick_x);

            //Toggles Parallel <--> Perpendicular
            if(controller1.a.onPress()){
                pivot.toggleMode();
            }

            //Moves Wrist (Through tuning Offsets)
            if(gamepad1.dpad_down){
                pivot.adjustOffset(-1);
            }else if(gamepad1.dpad_up){
                pivot.adjustOffset(1);
            }





            //Moves Arm
            pivot.adjustArm(controller1.left_trigger.getTriggerValue() - controller1.right_trigger.getTriggerValue());


            //Updates Wrist Position
            pivot.updatePos();
            controller1.update();

            telemetry.addLine("Use [a] to toggle between parallel and perpendicular");
            telemetry.addLine("Use [Triggers] to move Arm. Wrist should automatically follow");
            telemetry.addLine("Use [Dpad_Up] and [Dpad_Down] to tune the active offset variable\n");
            telemetry.addLine(pivot.toString());
            telemetry.update();
        }
    }
}
