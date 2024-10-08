package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="LiftArmClawTest", group = "Subsystem Tests")
public class LiftArmClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Init Phase
        Mecanum robot = new Mecanum(hardwareMap);

        Lift lift = new Lift(hardwareMap);

        Arm arm = new Arm(hardwareMap);

        Claw claw = new Claw(hardwareMap);

        GamepadEvents controller1 = new GamepadEvents(gamepad1);

        waitForStart();

        //Start Phase
        while(!isStopRequested()){

            //Drive Systems
            double forward = controller1.left_stick_y;
            double strafe = controller1.left_stick_x;
            double rotate = controller1.right_stick_x;
            robot.drive(forward,strafe,rotate);

            /*Subsystems:
                Notes:
                Use dynamic controls until constraint variables can be properly assigned/set
            * */

            //Lift Subsystem
            lift.moveLift(controller1.right_trigger.getTriggerValue() - controller1.left_trigger.getTriggerValue());


            //Arm Subsystem
            if (gamepad1.right_bumper){
                arm.changeHeight(1);
            }
            else if (gamepad1.left_bumper){
                arm.changeHeight(-1);
            }

            //Claw Subsystem
            if (gamepad1.a){
                claw.changePosition(1);
            } else if (gamepad2.b) {
                claw.changePosition(-1);
            }

            telemetry.addData("Lift Motor Position: ",lift.getPosition());
            telemetry.addData("Claw Servo Position: ",claw.getPosition());

            telemetry.update();
            controller1.update();
        }
    }
}
