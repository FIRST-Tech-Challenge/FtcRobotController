package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Competition", group = "Competition")
public abstract class TeleOpCompetition extends MasterTeleOp {

    int tickvalue = 97;

    //for run to position or manual control
    boolean toPosition = true;

    @Override
    public void runOpMode() {
        Initialize();
        resetArmAndServo();
        waitForStart();


        int addingticks = 0;
        boolean isPressed = false;
        double motorPower = 0.9;
        double increase = 1;

        //Set power of motors
        while (opModeIsActive()) {
            driver1.update();
            driver2.update();

            driveRobot();
            driveSlow();
            driveLeftCarousel();
            driveRightCarousel();
            driveGrabber();
            driveArm();
            driveArmManual();
            driveBelt();
            resetArmAndServo();

            //Use switch to declare values for each arm position
            switch (position) {
                case -1:
                    servoArm.setPosition(0.05);
                    motorArm.setPower(motorPower);
                    tickvalue = 650;
                    break;
                case 0:
                    servoArm.setPosition(0.45);
                    motorArm.setPower(motorPower);
                    tickvalue = 140;
                    break;
                case 1:
                    servoArm.setPosition(0.4);
                    motorArm.setPower(motorPower);
                    tickvalue = 300;
                    break;
                case 2:
                    servoArm.setPosition(0.3);
                    motorArm.setPower(motorPower);
                    tickvalue = 600;
                    break;
                case 3:
                    servoArm.setPosition(0.15);
                    motorArm.setPower(motorPower);
                    tickvalue = 900;
                    break;
                case 4:
                    servoArm.setPosition(0.1);
                    motorArm.setPower(motorPower);
                    tickvalue = 1000;
                    break;
                case 5:
                    servoArm.setPosition(0.4);
                    motorArm.setPower(motorPower);
                    tickvalue = 1980;
                    break;
                case 6:
                    servoArm.setPosition(0.25);
                    motorArm.setPower(motorPower);
                    tickvalue = 2310;
                    break;
                case 7:
                    servoArm.setPosition(0.4);
                    motorArm.setPower(motorPower);
                    tickvalue = 2330;
                    break;
            }

            // checks old position of arm, right when it goes over top of robot from front to back, it reduces speed
            if (gamepad2.dpad_up){
                if (!isPressed) {
                    position += increase;
                    if (position > 7){
                        position = 7;
                    }
                }
                addingticks = 0;
                toPosition = true;
                isPressed = true;
            }

            // checks old arm position, when it goes over the top of the robot from back to front, it reduces speed
            if (!gamepad2.dpad_up && !gamepad2.dpad_down){
                isPressed = false;
            } else if (gamepad2.dpad_down){
                if (!isPressed) {
                    position -= increase;
                    if (position < 0){
                        position = 0;
                    }
                }
                addingticks = 0;
                toPosition = true;
                isPressed = true;
            } else {
                motorPower = 0.5;
            }

            motorArm.setTargetPosition(tickvalue + addingticks);
            motorArm.setPower(0.9);
        }
    }
}