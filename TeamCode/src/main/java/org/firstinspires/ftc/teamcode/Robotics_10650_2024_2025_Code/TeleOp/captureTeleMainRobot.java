package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;

import java.io.FileWriter;
import java.util.ArrayList;

@TeleOp(name = "COPYRecordDriverInputs", group = "Linear Opmode")
public class captureTeleMainRobot extends LinearOpMode {
    RobotInitialize robot;
    int liftPitchPosition = 0;
    int liftExtenderPosition = 0;
    double maxLifEtxtension =0;


        private ArrayList<String> recordedInputs;

        // Variable for speed control
        private double speedMultiplier = 1.0;

        @Override
        public void runOpMode() {

//                robot = new ProgBotInitialize(this, false);
            robot = new RobotInitialize(this, false);


            // InitileftDrivealize hardware


                recordedInputs = new ArrayList<>();


                telemetry.addData("Status", "Ready to record driver inputs");
                telemetry.update();

                waitForStart();

                long startTime = System.currentTimeMillis();
                double strafeVelocity;
                double straightMovementVelocity;
                double turnVelocity;

                double fleftVel = 0;
                double bleftVel = 0;
                double frightVel= 0 ;
                double brightVel = 0;

                double pitchVel = 0;
                double extenderVel = 0;

                double intakeVel = 0;
                double clawPitchPos = 0;

                double hangRPos = 1;


                while (opModeIsActive()&&!gamepad1.share) {

                    // Adjust speed with Cross (X) or Circle




                    {
                        int speed = 2700;

                        if (gamepad1.circle) {
                            speed = 270;
                        }
                        if (gamepad1.cross) {
                            speed = 6969;
                        }

                        strafeVelocity = Math.pow(gamepad1.right_stick_x, 3) * speed;
                        //telemetry.addData("gamepad1.left_stick_x (strafing)", strafePower);
                        turnVelocity = Math.pow(gamepad1.left_stick_x, 3) * speed;
                        //telemetry.addData("gamepad1.right_stick_x (turning)", turnPower);
                        straightMovementVelocity = 0;
                        if (gamepad1.left_trigger > 0) {
                            //slow
                            straightMovementVelocity = Math.pow(gamepad1.left_trigger, 3) * speed;
                            //strafeVelocity = 0*(gamepad1.left_trigger);
                            //turnVelocity =0 * (gamepad1.left_trigger);

                        } else if (gamepad1.right_trigger > 0) {
                            //slow
                            straightMovementVelocity = -Math.pow(gamepad1.right_trigger, 3) * speed;
                            //strafeVelocity = 0*(gamepad1.right_trigger);
                            //turnVelocity = 0 *(gamepad1.right_trigger);

                        } else {
                            straightMovementVelocity = 0;
                        }

                        if (gamepad1.left_trigger > 0) {
                            //slow

                            straightMovementVelocity = 2700 * (gamepad1.left_trigger);
                            //strafeVelocity = 0*(gamepad1.left_trigger);
                            //turnVelocity =0 * (gamepad1.left_trigger);

                        }
                        if (gamepad1.right_trigger > 0) {
                            //slow
                            straightMovementVelocity = -2700 * (gamepad1.right_trigger);
                            //strafeVelocity = 0*(gamepad1.right_trigger);
                            //turnVelocity = 0 *(gamepad1.right_trigger);

                        }
                        if (gamepad1.circle) {
                            //slow
                            if (gamepad1.left_trigger > 0) {
                                straightMovementVelocity = 270;
                            } else if (gamepad1.right_trigger > 0) {
                                straightMovementVelocity = -270;

                            }
                            strafeVelocity = 270 * Math.signum(gamepad1.right_stick_x);
                            //turnVelocity = 0 * Math.signum(gamepad1.right_stick_x);
                            if (gamepad1.left_stick_x > 0) {
                                turnVelocity = 270 * Math.signum(gamepad1.left_stick_x);
                            } else if (gamepad1.left_stick_x > 0) {
                                turnVelocity = -270 * Math.signum(gamepad1.left_stick_x);
                            }
                        }
                        if (gamepad1.cross) {
                            //boost
                            if (gamepad1.left_trigger > 0) {
                                straightMovementVelocity = 6969;
                            } else if (gamepad1.right_trigger > 0) {
                                straightMovementVelocity = -6969;

                            }
                            strafeVelocity = 6969 * Math.signum(gamepad1.right_stick_x);

                            if (gamepad1.left_stick_x > 0) {
                                turnVelocity = 6969 * Math.signum(gamepad1.left_stick_x);
                            } else if (gamepad1.left_stick_x > 0) {
                                turnVelocity = -6969 * Math.signum(gamepad1.left_stick_x);
                            }
                        }
                        fleftVel = (strafeVelocity - straightMovementVelocity + turnVelocity);
                        frightVel = (-strafeVelocity - straightMovementVelocity - turnVelocity);
                        bleftVel = (strafeVelocity + straightMovementVelocity - turnVelocity);
                        brightVel = (-strafeVelocity + straightMovementVelocity + turnVelocity);

                        {
                            robot.fLeft.setVelocity(fleftVel); // Overall
                            // negative value
                            robot.fRight.setVelocity(frightVel); // Overall
                            // positive value
                            robot.bLeft.setVelocity(bleftVel); // Overall
                            // positive value
                            robot.bRight.setVelocity(brightVel); // Overall
                            // negative value
                        }
                    }
                    if (Math.abs(robot.liftPitch.getCurrentPosition()-liftPitchPosition)>50){
                        if (robot.liftPitch.getCurrentPosition()<liftPitchPosition){
                            pitchVel = 2250;
                            robot.liftPitch.setVelocity(2250);
                        } else if (robot.liftPitch.getCurrentPosition()>= liftPitchPosition) {
                            pitchVel = -2250;
                            robot.liftPitch.setVelocity(-2250);
                            if (liftPitchPosition>1500){
                                pitchVel = -2450;
                                robot.liftPitch.setVelocity(-2450);
                            }
                        }
                    } else {
                        robot.liftPitch.setVelocity(0);
                        pitchVel = 0;

                    }
                    telemetry.addData(" extender curent pos", robot.liftExtender.getCurrentPosition());
                    telemetry.addData("extender target pos", liftExtenderPosition);

                    //telemetry.addData("claw pitch pos",robot.clawRoll.getPosition());


                    telemetry.addData("is right stick pressed?", gamepad2.right_stick_button);
                    telemetry.addData("is left stick pressed?", gamepad2.left_stick_button);




//
                    telemetry.addData("ground mode pitch pos", Math.asin(53.78/robot.liftExtender.getCurrentPosition()));

//
                    if (liftPitchPosition<=2325&&liftPitchPosition>=0||
                            (liftPitchPosition>=2325&&gamepad2.left_stick_y > 0)|| // 3200 goes to the
                            // maximum horizontal position and further (try something less than this)
                            (liftPitchPosition<=0&&gamepad2.left_stick_y < 0)) {

                        if(liftExtenderPosition > maxLifEtxtension) {
                            liftExtenderPosition = (int) maxLifEtxtension;  //change to max lift xtension
                        }

                        if (gamepad2.left_stick_y > 0.2) {//going down
                            liftPitchPosition = liftPitchPosition - 25;

                            if (liftPitchPosition>1500){//if it low, go slow
                                liftPitchPosition = liftPitchPosition - 15;
                            }

                        } else if (gamepad2.left_stick_y< -0.2) {//going up
                            liftPitchPosition = liftPitchPosition + 45;//value used to be 25 justin case
                        }

                        if(liftPitchPosition < 0) {
                            liftPitchPosition = 0;
                        } else if(liftPitchPosition > 2300) {liftPitchPosition = 2300;  //change to max lift xtension
                        }

                    }


                    telemetry.addData("lift extender pos", robot.liftExtender.getCurrentPosition());
                    telemetry.addData("lift pitch pos", robot.liftPitch.getCurrentPosition());



                    //Bounds on the liftExtender motor
                    //Lift PIDF might be able to be tuned more to improve but it does reasonably well currently
                    double pitchAngle = robot.liftPitch.getCurrentPosition()*(90)/2595;




                    if (pitchAngle>=31.25){
                        maxLifEtxtension = 1210/(Math.sin(Math.toRadians(pitchAngle))); // horizontal bound
                    } else{
                        maxLifEtxtension = 2780;
                    }

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

                    if (pitchAngle>=31.25){
                        maxLifEtxtension = 1210/(Math.sin(Math.toRadians(pitchAngle))); // horizontal bound
                    } else{
                        maxLifEtxtension = 2780;

                    }




                    //if in bounnds, set new target pos


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
                    if (liftExtenderPosition<=(5)&&robot.liftExtender.getCurrentPosition()<=(5)) {
                        //when down, save power
                        extenderVel = 0;
                        robot.liftExtender.setVelocity(0);
                    }else if(Math.abs(robot.liftExtender.getCurrentPosition()-liftExtenderPosition)>25) {
                        //if far from target position

                        //next if own or up
                        if (robot.liftExtender.getCurrentPosition() < liftExtenderPosition) {
                            extenderVel = 1500;

                            robot.liftExtender.setVelocity(1500);
                        } else if (robot.liftExtender.getCurrentPosition() >= liftExtenderPosition) {
                            extenderVel = -1500;

                            robot.liftExtender.setVelocity(-1500);
                        }
                        //If no input, make sure the liftExtender motor does not move
                    }else {
                        extenderVel = 1;

                        robot.liftExtender.setVelocity(1);
                    }
                    if (gamepad2.left_trigger != 0) {
                        intakeVel = -1;
                        robot.intake.setPower(-1.0);
                        telemetry.addData("intake power", robot.intake.getPower());
                    }
                    else if (gamepad2.right_trigger != 0) {
                        intakeVel = 1;
                        robot.intake.setPower(1.0);
                        telemetry.addData("intake power", robot.intake.getPower());

                    } else {
                        intakeVel = 0;
                        robot.intake.setPower(0.0);
                        telemetry.addData("intake power", robot.intake.getPower());
                    }
                    if(gamepad2.dpad_down){

                        clawPitchPos = 0;
                        //robot.clawRoll.setPosition(0);

                    }
                    if(gamepad2.dpad_up){

                        clawPitchPos = 0.1606;
                        //robot.clawRoll.setPosition(0.1606);
                    }
                    if (gamepad1.dpad_left) {
                        robot.parkingServo.setPosition(robot.parkingServo.getPosition()+0.002);
                        hangRPos = robot.parkingServo.getPosition();



//                robot.hangR.setPosition(0.9611);
//                robot.hangL.setPosition(0.0439);
                    }
                    if (gamepad1.dpad_right) {
                        robot.parkingServo.setPosition(robot.parkingServo.getPosition()-0.002);
                        hangRPos = robot.parkingServo.getPosition();

//                robot.hangR.setPosition(0.9611);
//                robot.hangL.setPosition(0.0439);
                    }

                    // Record inputs with timestamp
                    recordedInputs.add((System.currentTimeMillis() - startTime) + "," + (fleftVel) + "," + (frightVel) + "," + (bleftVel) + "," + (brightVel)+","+ (extenderVel)+","+(pitchVel)+","+(intakeVel)+","+(clawPitchPos)+","+(hangRPos));

//                    telemetry.addData("Recording", "fleft Velocity: %.2f, fright Velocity: %.2f, bleft Velocity: %.2f, bright Velocity %.2f,", (startTime),(fleftVel),(frightVel),(bleftVel), (brightVel));
                    telemetry.update();
                }

                // Save recorded inputs after the session
                saveInputsToFile("/sdcard/FIRST/recordedInputsOnce.txt");

        }

        private void saveInputsToFile(String filename) {
            try (FileWriter writer = new FileWriter(filename)) {
                for (String input : recordedInputs) {
                    writer.write(input + "\n");
                }
                telemetry.addData("Status", "Inputs saved for future use");
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
            }
            telemetry.update();
        }
    }



