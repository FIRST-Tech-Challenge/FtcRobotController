package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;

@TeleOp(name = "correct")

public class teleCorrectLift extends LinearOpMode{
    RobotInitialize robot;
    int liftPitchPosition = 0;
    double liftExtenderPosition = 0;
    double maxLifEtxtension = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotInitialize(this, false);
        //robot.clawRoll.setPosition(0.1867);
        waitForStart();
        // int i = 0;
        while (opModeIsActive()) {
            if (Math.abs(robot.liftPitch.getCurrentPosition()-liftPitchPosition)>50){
                if (robot.liftPitch.getCurrentPosition()<liftPitchPosition){
                    robot.liftPitch.setVelocity(2150);
                } else if (robot.liftPitch.getCurrentPosition()>= liftPitchPosition) {
                    robot.liftPitch.setVelocity(-2150);
                    if (liftPitchPosition>1500){
                        robot.liftPitch.setVelocity(-2450);
                    }
                }
            } else {
                robot.liftPitch.setVelocity(0);
            }
            telemetry.addData(" extender curent pos", robot.liftExtender.getCurrentPosition());
            telemetry.addData("extender target pos", liftExtenderPosition);

            telemetry.addData("claw pitch pos",robot.clawRoll.getPosition());
            //
            if (liftPitchPosition<=2325&&liftPitchPosition>=-600||
                    (liftPitchPosition>=2325&&gamepad2.left_stick_y > 0)|| // 3200 goes to the
                    // maximum horizontal position and further (try something less than this)
                    (liftPitchPosition<=-600&&gamepad2.left_stick_y < 0)) {
                if(liftExtenderPosition > maxLifEtxtension) {
                    liftExtenderPosition = (int) maxLifEtxtension;  //change to max lift xtension
                }
                //determines where the lift pitch goes
                if (gamepad2.left_stick_y < -0.2) {//going up

                    liftPitchPosition = liftPitchPosition - 35;
                    if (liftPitchPosition>1500){
                        liftPitchPosition = liftPitchPosition - 25;
                    }


                } else if (gamepad2.left_stick_y>0.2) {//going down
                    liftPitchPosition = liftPitchPosition + 40;


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

                if(liftPitchPosition < -600) {
                    liftPitchPosition = -600;
                } else if(liftPitchPosition > 2300) {
                    liftPitchPosition = 2300;  //change to max lift xtension
                }



                //1300
            }//2700




            //lift pitch horizontal bounds

            //if pitch degree is less than 31.25

            //find positon for extension
            telemetry.addData("lift extender pos", robot.liftExtender.getCurrentPosition());
            telemetry.addData("lift extender target pos", liftExtenderPosition);

            telemetry.addData("lift pitch pos", robot.liftPitch.getCurrentPosition());



            //Bounds on the liftExtender motor
            //Lift PIDF might be able to be tuned more to improve but it does reasonably well currently
            double pitchAngle = robot.liftPitch.getCurrentPosition()*(90)/2595;
            if (pitchAngle>=31.25){
                maxLifEtxtension = 1210/(Math.sin(Math.toRadians(pitchAngle))); // horizontal bound
            } else{
                maxLifEtxtension = 3100;

            }


            //the button removes bounds when pressed
//            if (gamepad2.right_stick_button){ //test these
//                //removes bounds
//                minLiftExtension = -2300;
//                maxLifEtxtension = 40000;
//            } else{ //make sure bounds are active if the button is not pressed


//            }

            if (gamepad2.left_stick_button){ //when this button is pressed
                robot.liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //press this button to set the zero point after adjusting with right stick button
            } else{
                robot.liftExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            telemetry.addData("max lift etension", 1567/(Math.sin(Math.toRadians(pitchAngle))));
            telemetry.addData("cos", (Math.sin(Math.toRadians(pitchAngle))));
            telemetry.addData("pitch angle", (robot.liftPitch.getCurrentPosition()*90)/2595);
            telemetry.addData("pitch angle", pitchAngle);






            //if in bounnds, set new target pos

            //used to be ((Math.abs(gamepad2.right_stick_y)>0.2)&&(liftExtenderPosition<=maxLifEtxtension)
            //                    &&(liftExtenderPosition>=0)||(robot.liftExtender.getCurrentPosition()<0&&
            //                    gamepad2.right_stick_y<0)||(robot.liftExtender.getCurrentPosition()>
            //                    maxLifEtxtension&&gamepad2.right_stick_y>0))
            if ((Math.abs(gamepad2.right_stick_y)>0.2)&&(liftExtenderPosition<=maxLifEtxtension)
                    &&(liftExtenderPosition>=0)||(robot.liftExtender.getCurrentPosition()<0&&
                    gamepad2.right_stick_y<0)||(robot.liftExtender.getCurrentPosition()>
                    maxLifEtxtension&&gamepad2.right_stick_y>0)) {

                liftExtenderPosition = liftExtenderPosition - (int)(30*gamepad2.right_stick_y);


                if(liftExtenderPosition < 0) {
                    liftExtenderPosition = 0;
                } else if(liftExtenderPosition > maxLifEtxtension) {
                    liftExtenderPosition = (int) maxLifEtxtension;  //change to max lift xtension
                }

            }


            //Determines if the liftExtender should go up or down based on the controller inputs
            if (liftExtenderPosition<=(5)&&robot.liftExtender.getCurrentPosition()<=(5)&&!gamepad2.right_bumper) {
                //when down, save power
                robot.liftExtender.setVelocity(0);
            }else if(Math.abs(robot.liftExtender.getCurrentPosition()-liftExtenderPosition)>25) {
                //if far from target position

                //next if own or up
                if (robot.liftExtender.getCurrentPosition() < liftExtenderPosition) {
                    robot.liftExtender.setVelocity(1500);
                } else if (robot.liftExtender.getCurrentPosition() >= liftExtenderPosition) {
                    robot.liftExtender.setVelocity(-1500);
                }
                //If no input, make sure the liftExtender motor does not move
            }else {
                robot.liftExtender.setVelocity(1);
            }

            if (gamepad2.left_bumper){
                robot.pitch.setPosition(robot.pitch.getPosition()+0.005);
                telemetry.addData("pitch is moving", 1);
            }
            if (gamepad2.left_trigger>0){
                robot.pitch.setPosition(robot.pitch.getPosition()-0.005);
                telemetry.addData("pitch is moving", 1);

            }

            if (gamepad2.right_bumper){
                robot.clawRoll.setPosition(robot.clawRoll.getPosition()+0.005);
                telemetry.addData("roll is moving", 1);

            }
            if (gamepad2.right_trigger>0){
                robot.clawRoll.setPosition(robot.clawRoll.getPosition()-0.005);
                telemetry.addData("roll is moving", 1);

            }

            telemetry.addData("pitch position", robot.pitch.getPosition());
            telemetry.addData("roll position", robot.clawRoll.getPosition());






            telemetry.update();
        }
    }
}


