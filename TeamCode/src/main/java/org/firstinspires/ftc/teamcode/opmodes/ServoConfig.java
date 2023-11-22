package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@TeleOp
public class ServoConfig extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        waitForStart();

        while (!isStopRequested()) {
            //EDWARD'S INTAKE
            boolean buttonA = gamepad2.a;
            boolean buttonB = gamepad2.b;
            //telemetry.addData("left_stick_y:", gamepad1.left_stick_y);
            telemetry.update();
            robot.update();

            /*
            if(buttonA) {
                robot.intake.setPower(1.0);
            }
            if(buttonB){
                robot.intake.setPower(0);
            }
            robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x));
            */

            //UG OUTTAKE
            boolean dpadUp = gamepad1.dpad_up;//move both +
            boolean dpadDown = gamepad1.dpad_down;//move both arms -
            boolean dpadRight = gamepad1.dpad_right;//adjust right arm +
            boolean dpadLeft = gamepad1.dpad_left;//adjust right arm-

            if(dpadUp){
                robot.outtake.moveArm(0.1);
            } if(dpadDown) {
                robot.outtake.moveArm(-0.1);
            } if(dpadRight){
                robot.outtake.moveR(0.1);
            } if(dpadLeft) {
                robot.outtake.moveR(-0.1);
            } if (gamepad1.x){
                robot.outtake.resetPos();
            }

            telemetry.addData("right servo position: ", robot.outtake.getRightServoPos());
            telemetry.addData("left servo position: ", robot.outtake.getLeftServoPos());
            //Log.v("arm", "right servo position: "+ robot.outtake.getRightServoPos());

        }
    }
}
