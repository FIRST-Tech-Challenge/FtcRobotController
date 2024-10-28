package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "corect")

public class teleCorrectLift extends LinearOpMode{
    RobotInitialize robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotInitialize(this);
        //robot.clawRoll.setPosition(0.1867);
        waitForStart();
       // int i = 0;
        while(opModeIsActive()){
            if (Math.abs(gamepad2.right_stick_x)>0.2) {
                robot.liftPitch.setVelocity(400*gamepad2.right_stick_x);
                telemetry.addData("lift pitchpos", robot.liftPitch.getCurrentPosition());

            } else{
                robot.liftPitch.setVelocity(0);
            }//1300
            //2700
            //find positon for extension
            if (Math.abs(gamepad2.right_stick_y)>0.2) {
                robot.liftExtender.setVelocity(-400*gamepad2.right_stick_y);
                telemetry.addData("extenderhpos", robot.liftExtender.getCurrentPosition());
            } else{
                robot.liftExtender.setVelocity(0);
            }

            telemetry.addData("extenderhpos", robot.liftExtender.getCurrentPosition());
            telemetry.addData("lift pitchpos", robot.liftPitch.getCurrentPosition());

            //robot.clawRoll.setPosition(robot.clawRoll.getPosition()+ gamepad1.left_stick_x*0.0001);
           //robot.pitch.setPosition(robot.pitch.getPosition()+ gamepad1.right_stick_x*0.0001);

           // telemetry.addData("clawRoll Pos", robot.clawRoll.getPosition());
            //telemetry.addData("clawPitch Pos",robot.pitch.getPosition());

//            robot.liftPitch.setVelocit(500*gamepad2.right_stick_y);
            robot.clawRoll.setPosition(robot.clawRoll.getPosition()+ (gamepad1.left_stick_x*0.0001));

            robot.pitch.setPosition(robot.pitch.getPosition()+ (gamepad2.left_stick_x*0.0001));

            //i=i+Math.round(gamepad2.right_stick_y);

           telemetry.addData("clawpos", robot.clawRoll.getPosition());
           telemetry.addData("pitchpos", robot.pitch.getPosition());

            telemetry.update();
        }
    }
}

