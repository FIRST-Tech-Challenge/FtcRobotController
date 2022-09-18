package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Autonomous(name="hello world")
@Disabled
public class hello_world extends LinearOpMode{
    private ElapsedTime     runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        TurtleRobot robot = new TurtleRobot(this);
        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                    robot.rightfrontmotor.setPower(1);
                    robot.leftfrontmotor.setPower(1);
                    robot.rightbackmotor.setPower(1);
                    robot.leftbackmotor.setPower(1);
                }
                robot.rightfrontmotor.setPower(0);
                robot.leftfrontmotor.setPower(0);
                robot.rightbackmotor.setPower(0);
                robot.leftbackmotor.setPower(0);
            }
        }
    }
}
