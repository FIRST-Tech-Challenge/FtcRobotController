package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "correct")

public class teleCorrectLift extends LinearOpMode{
    RobotInitialize robot;
    int liftPitchPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotInitialize(this);
        //robot.clawRoll.setPosition(0.1867);
        waitForStart();
        // int i = 0;
        while (opModeIsActive()) {
            if (Math.abs(robot.liftPitch.getCurrentPosition() - liftPitchPosition) > 50) {
                if (robot.liftPitch.getCurrentPosition() < liftPitchPosition) {
                    robot.liftPitch.setVelocity(2250);
                } else if (robot.liftPitch.getCurrentPosition() >= liftPitchPosition) {
                    robot.liftPitch.setVelocity(-2250);
                    if (liftPitchPosition > 1500) {
                        robot.liftPitch.setVelocity(-2450);
                    }
                }
            } else {
                robot.liftPitch.setVelocity(0);
            }
            telemetry.addData(" liftpitch curent pos", robot.liftPitch.getCurrentPosition());
            //telemetry.addData("extender target pos", liftExtenderPosition);

            telemetry.addData("claw pitch pos", robot.clawRoll.getPosition());
            //telemetry.addData("Pitch TargetPos",liftPitchPosition);

//            if (gamepad1.left_bumper) {
//                //edit this to be valid for the dual mode servo
//                robot.clawRoll.setPosition(robot.clawRoll.getPosition()+ 0.001);
//            }
//            if (gamepad1.right_bumper) {
//                //edit this to be valid for the dual mode servo
//                robot.clawRoll.setPosition(robot.clawRoll.getPosition()- 0.001);
//            }
//
//            if (gamepad2.cross) {
//                robot.liftExtender(0, 0.3);
//            }
            //telemetry.addData("roll", robot.clawRoll.getPosition());
//            int pitchSpeed  = 25;
//            //telemetry.addData("joystick pos", gamepad2.left_stick_y);
//            if (gamepad2.right_bumper){
//                pitchSpeed = 10;
//            } else if (!gamepad2.right_bumper){
//                pitchSpeed = 25;
//            }
            if (liftPitchPosition <= 3000 && liftPitchPosition >= 0 ||
                    (liftPitchPosition >= 3000 && gamepad2.left_stick_y > 0) || // 3200 goes to the
                    // maximum horizontal position and further (try something less than this)
                    (liftPitchPosition <= 0 && gamepad2.left_stick_y < 0)) {

                //determines where the lift pitch goes
                if (gamepad2.left_stick_y > 0.2) {//going down

                    liftPitchPosition = liftPitchPosition - 25;
                    if (liftPitchPosition > 1500) {
                        liftPitchPosition = liftPitchPosition - 15;
                    }


                } else if (gamepad2.left_stick_y < -0.2) {//going up
                    liftPitchPosition = liftPitchPosition + 25;


                    //if it is a t a really low point


//                    if (robot.liftPitch.getCurrentPosition()<400) {
//                        robot.liftPitch.setVelocity(470 * gamepad2.left_stick_y);
//                    }else if(robot.liftPitch.getCurrentPosition()>600){
//                        robot.liftPitch.setVelocity(1550 * gamepad2.left_stick_y);
//
//
//                    } else{
//                        robot.liftPitch.setVelocity(800 * gamepad2.left_stick_y);
//                        telemetry.addData("left pitch pos", robot.liftPitch.getCurrentPosition());
//
//                    }

                }

                if (liftPitchPosition < 0) {
                    liftPitchPosition = 0;
                } else if (liftPitchPosition > 3000) {
                    liftPitchPosition = 3000;  //change to max lift xtension
                }

//                telemetry.addData("extenderhpos", robot.liftExtender.getCurrentPosition());
                telemetry.addData("lift pitchpos", robot.liftPitch.getCurrentPosition());

                //robot.clawRoll.setPosition(robot.clawRoll.getPosition()+ gamepad1.left_stick_x*0.0001);
                //robot.pitch.setPosition(robot.pitch.getPosition()+ gamepad1.right_stick_x*0.0001);

                // telemetry.addData("clawRoll Pos", robot.clawRoll.getPosition());
                //telemetry.addData("clawPitch Pos",robot.pitch.getPosition());

//            robot.liftPitch.setVelocit(500*gamepad2.right_stick_y);
//                robot.hangR.setPosition(robot.hangR.getPosition() + (gamepad1.right_stick_x * 0.01));
//
//                robot.hangL.setPosition(robot.hangL.getPosition() + (gamepad1.left_stick_x * 0.001));
//
//
//                telemetry.addData("hang r pos", robot.clawRoll.getPosition());
//                telemetry.addData("hang l pos", robot.pitch.getPosition());

                telemetry.update();
            }
        }
    }
}

