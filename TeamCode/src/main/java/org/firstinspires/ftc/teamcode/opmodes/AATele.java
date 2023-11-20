package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
@TeleOp
public class AATele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        waitForStart();

        while (!isStopRequested()) {

            boolean buttonA = gamepad2.a;
            boolean buttonB = gamepad2.b;
            //telemetry.addData("left_stick_y:", gamepad1.left_stick_y);
            telemetry.update();
            robot.update();

            if(buttonA) {
                robot.intake.setPower(1.0);
            }
            if(buttonB){
                robot.intake.setPower(0);
            }
            robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x));



        }
    }
}
