// Program created by: Danny and William
// Purpose: FTC Robot Software

package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;

@TeleOp (name = "TeleOp_RobotCentric")
public class TeleOpCode_RobotCentric extends LinearOpMode {

    // Run the initialize function
    RobotInitialize robot;

    int liftPitchPosition = 0;
    int liftExtenderPosition = 0;
    double maxLifEtxtension = 0;
    double targetPitchVert = 0;

    final double liftDist = 8.25;


    @Override
    public void runOpMode() throws InterruptedException {
// create and define the initialization variable
        robot = new RobotInitialize(this, false);

        // initialization of the control of the robot when start is pressed
        waitForStart();

        //edit this to be valid for the dual mode servo
        //initial position
        robot.parkingServo.setPosition(1);

        //robot.intake.setPower(0);


        // loop while the program is running
        // waits for controller input then runs the associated code
        while (opModeIsActive()) {
            // controller inputs that is inputted by the drive team
            controllerInput();
        }
    }

    public void controllerInput() {
        //robot.intake.setPower(0);

        // Gamepad usages (two gamepads in use, one for driving and one for mechanisms):

        // Gamepad1 is used for driving (motor controls)
        // Gamepad2 is used for mechanism manipulation (moving servos and the lift motors)


        // Variables that store the different game pad movements for ease of reference later
        // Gamepad1 configuration
        {
            int speed = 2700;

            if (gamepad1.right_bumper) {
                robot.parkingServo.setPosition(0.946); //Where it can touch the bar
            } else {
                robot.parkingServo.setPosition(1); //all the way down
            }

            if (gamepad1.circle) {
                speed = 270;
            }
            if (gamepad1.cross) {
                speed = 6969;
            }

            double strafeVelocity; // (left stick x-axis movement)
            strafeVelocity = Math.pow(gamepad1.right_stick_x, 3) * speed; // Min: -10000, Max: 10000
            //telemetry.addData("gamepad1.left_stick_x (strafing)", strafePower);
            double turnVelocity; // (right stick x-axis movement)
            turnVelocity = Math.pow(gamepad1.left_stick_x, 3) * speed; // Min: -10000, Max: 10000
            //telemetry.addData("gamepad1.right_stick_x (turning)", turnPower);
            double straightMovementVelocity = 0;
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
            ; // (left stick y-axis movement)
//      straightMovementPower = 10000*(gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y);
// Min: -10000, Max: 10000
            //original: straightMovementVelocity = Math.pow(gamepad1.right_stick_y, 3) * 10000;
            //telemetry.addData("gamepad1.left_stick_y (straight movement)", strafePower);
            //Gamepad1 controls the drivetrain

            //not sure what this does
//            if (gamepad1.circle) {
//                straightMovementVelocity = Math.pow(gamepad1.right_stick_y, 3) * 1000;
//                turnVelocity = Math.pow(gamepad1.left_stick_x, 3) * 1000;
//                strafeVelocity = Math.pow(gamepad1.right_stick_x, 3) * 1000;
//                telemetry.addData("L2 pos", gamepad1.left_trigger);
//                telemetry.update();
//            }
        /*if(gamepad1.y){
            //testing upper bound of lift
            robot.liftExtender.setPower(.25);
            telemetry.addData("position", robot.liftExtender.getCurrentPosition());
            telemetry.update();
        }*/
//            if (gamepad1.right_trigger != 0) {
//                //normal speed
//                //after testing: it went backward sby accident
//                straightMovementVelocity = 400 * Math.signum(gamepad1.right_trigger);
//                strafeVelocity = 400 * Math.signum(gamepad1.right_stick_x);
//                turnVelocity = 400 * Math.signum(gamepad1.right_stick_x);
//            }

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


            // Set velocity of the motors (drivetrain)
            // Forward and backward movement (left stick y-axis movement)
            // Left and right turning (right stick x-axis movement)
            // Strafing left and right (left stick x-axis movement)
            {
                robot.fLeft.setVelocity(strafeVelocity - straightMovementVelocity + turnVelocity); // Overall
                // negative value
                robot.fRight.setVelocity(-strafeVelocity - straightMovementVelocity - turnVelocity); // Overall
                // positive value
                robot.bLeft.setVelocity(strafeVelocity + straightMovementVelocity - turnVelocity); // Overall
                // positive value
                robot.bRight.setVelocity(-strafeVelocity + straightMovementVelocity + turnVelocity); // Overall
                // negative value
            }
        }

        // Gamepad2 configuration
        {

//            if(gamepad2.left_bumper){//emergency button stops all movement (we can change that actual button later)
//                liftPitchPosition =robot.liftPitch.getCurrentPosition();
//                liftExtenderPosition =robot.liftPitch.getCurrentPosition();
//
//            }

            //determines the speed
            if (Math.abs(robot.liftPitch.getCurrentPosition() - liftPitchPosition) > 50) {
                if (robot.liftPitch.getCurrentPosition() < liftPitchPosition) {
                    robot.liftPitch.setVelocity(2150);
                } else if (robot.liftPitch.getCurrentPosition() >= liftPitchPosition) {
                    robot.liftPitch.setVelocity(-2150);
                    if (liftPitchPosition > 1500) {
                        robot.liftPitch.setVelocity(-3000);
                    }
                }
            } else {
                robot.liftPitch.setVelocity(0);
            }
            telemetry.addData(" extender curent pos", robot.liftExtender.getCurrentPosition());
            telemetry.addData("extender target pos", liftExtenderPosition);


            //telemetry.addData("Pitch TargetPos",liftPitchPosition);

//            if (gamepad1.left_bumper) {
//                //edit this to be valid for the dual mode servo
//            }
//            if (gamepad1.right_bumper) {
//                //edit this to be valid for the dual mode servo
//            }
//
//            if (gamepad2.cross) {
//                robot.liftExtender(0, 0.3);
//            }

//            int pitchSpeed  = 25;
//            //telemetry.addData("joystick pos", gamepad2.left_stick_y);
//            if (gamepad2.right_bumper){
//                pitchSpeed = 10;
//            } else if (!gamepad2.right_bumper){
//                pitchSpeed = 25;

//            }


            telemetry.addData("is right stick pressed?", gamepad2.right_stick_button);
            telemetry.addData("is left stick pressed?", gamepad2.left_stick_button);


//            if (gamepad2.share) {
//                telemetry.addData("is share pressed?", "yes");
//                robot.liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            } //else{
            //robot.liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.liftExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Needs to not reset once teleop begins

            //}

//            if (gamepad2.options) {
//                minLiftExtension = -2300;
//                telemetry.addData("is options pressed?", "yes");
//
//            }else{
//                minLiftExtension = 0;
//
//            }

//                robot.liftExtender(0, 0.3);

//            }
            telemetry.addData("ground mode pitch pos", Math.asin(53.78 / robot.liftExtender.getCurrentPosition()));

//            if (gamepad2.circle){
//                telemetry.addData("ground mode pitch pos", Math.asin(53.78/robot.liftExtender.getCurrentPosition()));
//
//
//            }
            if (liftPitchPosition <= 2325 && liftPitchPosition >= -600 ||
                    (liftPitchPosition >= 2325 && gamepad2.left_stick_y > 0) || // 3200 goes to the
                    // maximum horizontal position and further (try something less than this)
                    (liftPitchPosition <= -600 && gamepad2.left_stick_y < 0)) {
                if (liftExtenderPosition > maxLifEtxtension) {
                    liftExtenderPosition = (int) maxLifEtxtension;  //change to max lift xtension
                }
                //determines where the lift pitch goes
                if (gamepad2.left_stick_y < -0.2) {//going up

                    liftPitchPosition = liftPitchPosition - 35;
                    if (liftPitchPosition > 1500) {
                        liftPitchPosition = liftPitchPosition - 25;
                    }


                } else if (gamepad2.left_stick_y > 0.2) {//going down
                    liftPitchPosition = liftPitchPosition + 40;


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

                if (liftPitchPosition < -600) {
                    liftPitchPosition = -600;
                } else if (liftPitchPosition > 2300) {
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
            double pitchAngle = robot.liftPitch.getCurrentPosition() * (90) / 2595;
            if (pitchAngle >= 31.25) {
                maxLifEtxtension = 1210 / (Math.sin(Math.toRadians(pitchAngle))); // horizontal bound
            } else {
                maxLifEtxtension = 5000;

            }


            //the button removes bounds when pressed
//            if (gamepad2.right_stick_button){ //test these
//                //removes bounds
//                minLiftExtension = -2300;
//                maxLifEtxtension = 40000;
//            } else{ //make sure bounds are active if the button is not pressed


            // if (pitchAngle>=31.25){
            // maxLifEtxtension = 1210/(Math.sin(Math.toRadians(pitchAngle))); // horizontal bound
            // } else{
            // maxLifEtxtension = 2780;
            //}

            if (gamepad2.left_stick_button) { //when this button is pressed
                robot.liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //press this button to set the zero point after adjusting with right stick button
            } else {
                robot.liftExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            telemetry.addData("max lift etension", 1567 / (Math.sin(Math.toRadians(pitchAngle))));
            telemetry.addData("cos", (Math.sin(Math.toRadians(pitchAngle))));
            telemetry.addData("pitch angle", (robot.liftPitch.getCurrentPosition() * 90) / 2595);
            telemetry.addData("pitch angle", pitchAngle);


            //if in bounnds, set new target pos

            //used to be ((Math.abs(gamepad2.right_stick_y)>0.2)&&(liftExtenderPosition<=maxLifEtxtension)
            //                    &&(liftExtenderPosition>=0)||(robot.liftExtender.getCurrentPosition()<0&&
            //                    gamepad2.right_stick_y<0)||(robot.liftExtender.getCurrentPosition()>
            //                    maxLifEtxtension&&gamepad2.right_stick_y>0))
            if ((Math.abs(gamepad2.right_stick_y) > 0.2) && (liftExtenderPosition <= maxLifEtxtension)
                    && (liftExtenderPosition >= 0) || (robot.liftExtender.getCurrentPosition() < 0 &&
                    gamepad2.right_stick_y < 0) || (robot.liftExtender.getCurrentPosition() >
                    maxLifEtxtension && gamepad2.right_stick_y > 0)) {

                liftExtenderPosition = liftExtenderPosition - (int) (30 * gamepad2.right_stick_y);


                if (liftExtenderPosition < 0) {
                    liftExtenderPosition = 0;
                } else if (liftExtenderPosition > maxLifEtxtension) {
                    liftExtenderPosition = (int) maxLifEtxtension;  //change to max lift xtension
                }

            }


            //Determines if the liftExtender should go up or down based on the controller inputs
            if (liftExtenderPosition <= (5) && robot.liftExtender.getCurrentPosition() <= (5) && !gamepad2.right_bumper) {
                //when down, save power
                robot.liftExtender.setVelocity(0);
            } else if (Math.abs(robot.liftExtender.getCurrentPosition() - liftExtenderPosition) > 25) {
                //if far from target position

                //next if own or up
                if (robot.liftExtender.getCurrentPosition() < liftExtenderPosition) {
                    robot.liftExtender.setVelocity(1500);
                } else if (robot.liftExtender.getCurrentPosition() >= liftExtenderPosition) {
                    robot.liftExtender.setVelocity(-1500);
                }
                //If no input, make sure the liftExtender motor does not move
            } else {
                robot.liftExtender.setVelocity(1);
            }
//
            //ejaculates the block
            if (gamepad2.left_trigger != 0) {
                robot.intake.setPower(-1.0);
                telemetry.addData("intake power", robot.intake.getPower());
            } else if (gamepad2.right_trigger != 0) {
                robot.intake.setPower(1.0);
                telemetry.addData("intake power", robot.intake.getPower());

            } else {
                robot.intake.setPower(0.0);
                telemetry.addData("intake power", robot.intake.getPower());
            }

            if (gamepad2.dpad_down) {

                robot.pitch.setPosition(0);

            }
            if (gamepad2.dpad_up) {

                robot.pitch.setPosition(0.1606/5);
            }

            if (gamepad1.dpad_left) {
                robot.parkingServo.setPosition(robot.parkingServo.getPosition() + 0.002);

//                robot.hangR.setPosition(0.9611);
//                robot.hangL.setPosition(0.0439);
            }
            if (gamepad1.dpad_right) {
                robot.parkingServo.setPosition(robot.parkingServo.getPosition() - 0.002);

//                robot.hangR.setPosition(0.9611);
//                robot.hangL.setPosition(0.0439);
            }
            telemetry.addData("hang r pos", robot.parkingServo.getPosition());

//            if (gamepad2.circle) {
//                before 2167
//                liftPitchPosition = 2090;
//
//                //edit this to be valid for the dual mode servo
//            }

            //edit this to be valid for the dual mode servo
        }

        if (gamepad2.square) {//slaps it in
            liftExtenderPosition = 0;
            liftPitchPosition = 272;
        }

        if (gamepad2.triangle) {//to score hifh basket
//                robot.liftPitch(0, 0.2);
//                telemetry.addData("Pitchpos", robot.liftPitch.getCurrentPosition());
            liftPitchPosition = 272;
            liftExtenderPosition = 2502;
        }
        //up pos = 0.3372
//            if (gamepad2.left_bumper){
//            } if (gamepad1.square){
//                robot.hangL.setPosition(robot.hangL.getPosition()+ (0.002));
//            }
//            if (gamepad2.right_bumper){
//            } //if (gamepad1.triangle)
//              robot.hangL.setPosition(robot.hangL.getPosition()- (0.002));
//            }

        //i=i+Math.round(gamepad2.right_stick_y);

        //telemetry.addData("hang l pos", robot.hangL.getPosition());


        // accelerationAdditive is 1428.57
        // The intended result is that when the control sticks are not stationary the speed slowly
        // increases until it gets to the max value of 10000 or -10000

        /*
        if encoder value too diff from initial, then increase speed
        as change in position increases, speed increases

        diff int = currentpos-initpos
        speed

        */

//        ElapsedTime accelerationTime = new ElapsedTime();
//
//        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0|| gamepad1.right_stick_x != 0) {
//            for (int i = 0; i < 7; i++) {
//                strafePower = gamepad1.left_stick_x * 1428.57;
//                turnPower = gamepad1.right_stick_x * 1428.57;
//                straightMovementPower = gamepad1.left_stick_y * 1428.57;
//            }
//        }

        // Prints to the robot driver station screen
        telemetry.update();
    }
}
//}