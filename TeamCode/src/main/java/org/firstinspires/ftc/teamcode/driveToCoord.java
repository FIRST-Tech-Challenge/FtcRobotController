package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//TICKS PER INCH = 752

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//system for driving to coordinates

//this may go in the main autonomous file due to its amount of interactions with the hardware

// because the control hub is tilted, the heading we use may not be yaw (z-coord) and may be the x or y coords

/*
OTHER IMPORTANT NOTES:

The startingHeading variable is the angle relative to the field, as if North was 90. It
needs to be inputted at the beginning of the round so the robot knows where it's actually facing.

Remember that ccw rotation increases yaw
 */

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import android.media.Image;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Autonomous(name="driveToCoord")
public class driveToCoord extends LinearOpMode {

    //this is filled in from teleOp
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor LFL;
    private DcMotor LFR;

    private Servo Push;

    private Servo Plane;

    private IMU imu;

    String telemetryMessage = "Enter Starting Heading:";

    //dimensions are 360*360, actual is 12*12ft on real field
    final int fieldX = 360;
    final int fieldY = 360;

    final int ticksPerInch = 4; //guess

    //not using tickoffse
    int ticksOffset = 0; //set this to the current tick position and subtract it later to get the ticks since the time you most recently changed it

    int tickAverage;

    double startingHeading; //use imu at init for this <- DO NOT I WAS DUMB
    //Starting heading and yaw will both be 0, this should be input by humans for the heading relative to field.
    //startingheading is like an offset, because if the robot starts pointed south then the yaw will be 0 even though
    //relative to field its 270
    double yaw;

    //relative to field, not robot (that is yaw)
    double currentFieldHeading = yaw + startingHeading;


    //the heading (yaw) angle at which the robot is pointed toward's the field's North
    final double trueNorth = 90 - startingHeading;

    YawPitchRollAngles robotOrientation;

    //important locations:
    class importantLocation {


    }










    double[] currentCoordinates = new double[2];

    //using array so that we can return a coordinate pair.
    //normally, this method is in reverse: we give it x and y coords to go to and it calculates the heading and ticks needed to go there
    //since many local variables ahve same names as global scope variables, 'f' means it is part of that method







    //IMU YAW IS 180 TO -180. THIS CURRENTLY POSSIBLY MESSES EVERYTHING UP!! circle is 0-360, yaw is 180 to -180
    //starting yaw is 0. This should mean that any rotation can be achived







//f means specific to this method

    //used to convert the values we are using to the ones the yaw will output and the reverse
    //we use one that goes from 0 to 360 ccw. yaw is 180 to -180 ccw

    public double reverseConvert(double practicalAngle){
        return practicalAngle - 180;
    }
    //reverse. converts yaw values into practical, usable values.
    public double convertAngle(double yawf) {
        return yawf + 180;
    }
    public double[] findLocation(double startingHeadingf, double currentXf, double currentYf,
                                 double distanceTraveledf, double yawf) {
        //starting heading will probably be 90, 180 or 0
        currentFieldHeading = yawf + startingHeadingf;
        //go to mathisfun unit circle if confused
        double newX = currentXf + distanceTraveledf * cos(currentFieldHeading);
        double newY = currentYf + distanceTraveledf * sin(currentFieldHeading);
        currentCoordinates[0] = newX;
        currentCoordinates[1] = newY;
        return currentCoordinates;
    }

    //moves robot to specificed coords
    public void driveToCoord(double trueNorth, double currentX, double currentY, double goalX,
                             double goalY) {

        robotOrientation = imu.getRobotYawPitchRollAngles();
        //may need to be a different measure b/c control hub is not deafult oriented, but may be fine b/c initialized with orientation
        yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        //while loop rotates robot until its pointed north (90 degrees)
        //margin of error may be necessary
        while (currentFieldHeading != 90) {
            BL.setPower(.2);
            FL.setPower(.2);
            BR.setPower(-.2);
            FR.setPower(-.2);
            //update yaw and field heading
            robotOrientation = imu.getRobotYawPitchRollAngles();
            yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            currentFieldHeading = yaw + startingHeading;
            //UPDATE currentFieldHeading AT END OF METHOD
            if (currentFieldHeading == 90) {
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
            }
        }
        //differnce between current and goal coords
        //may need to be absolute value so trig functions return positive value
        double distanceX = goalX - currentX;
        double distanceY = goalY - currentY;
        double hypotenuse = sqrt(distanceX * distanceX + distanceY * distanceY);
        ///ouptput of arctan restricted to {90 < x < -90}, so this may need to be modified
        //+180 means that if triangle is in quadrant 2 & 3 the angle should be correct, meaning that is the difference in the angle from
        //90 field heading, which is trueNorth

        ///the following if statements rotate robot to accurate heading, then it will drive a certain sidstance forward
        //based on ticks per inch of wheel encoders (not implemented yet)
        //they may need a conditional in the while loops to set power to 0 when goalHeading is reached

        //quadrant 3 & 2 (negative output for 2, too small ouput for 3)
        if ((distanceX < 0 && distanceY > 0) || (distanceX < 0 && distanceY < 0)) { //left side of || is quadrant 2
            double goalFieldHeading = (atan(distanceY / distanceX)) + 180; //atan returns -value for q2 and ppsiitve for q3
            //rotates until achieves goal heading (margin of error maybe necessary
            while (currentFieldHeading != goalFieldHeading) {
                BL.setPower(.2);
                FL.setPower(.2);
                BR.setPower(-.2);
                FR.setPower(-.2);
                //update yaw and field heading
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                currentFieldHeading = yaw + startingHeading;
                //UPDATE currentFieldHeading AT END OF METHOD
                if (currentFieldHeading == 90) {
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    BR.setPower(0);
                }
            }

        }
        //quadrant 4
        else if (distanceX > 0 && distanceY < 0) {
            double goalFieldHeading = (atan(distanceY / distanceX)) + 360;
            //rotates until achieves goal heading (margin of error maybe necessary
            while (currentFieldHeading != goalFieldHeading) {
                BL.setPower(.2);
                FL.setPower(.2);
                BR.setPower(-.2);
                FR.setPower(-.2);
                //update yaw and field heading
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                currentFieldHeading = yaw + startingHeading;
                //UPDATE currentFieldHeading AT END OF METHOD
                if (currentFieldHeading == 90) {
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    BR.setPower(0);
                }
            }
        }
        //quadrant 1
        else if (distanceX > 0 && distanceY > 0) {
            double goalFieldHeading = atan(distanceY / distanceX);
            while (currentFieldHeading != goalFieldHeading) {
                BL.setPower(.2);
                FL.setPower(.2);
                BR.setPower(-.2);
                FR.setPower(-.2);
                //update yaw and field heading
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                currentFieldHeading = yaw + startingHeading;
                //UPDATE currentFieldHeading AT END OF METHOD
                if (currentFieldHeading == 90) {
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    BR.setPower(0);
                }
            }
        }
        //straight north
        else if (distanceX == 0 && distanceY > 0) {
            double goalFieldHeading = 90; //North, robot should already be pointed there
            //rotate
            while (currentFieldHeading != goalFieldHeading) {
                BL.setPower(.2);
                FL.setPower(.2);
                BR.setPower(-.2);
                FR.setPower(-.2);
                //update yaw and field heading
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                currentFieldHeading = yaw + startingHeading;
                //UPDATE currentFieldHeading AT END OF METHOD
                if (currentFieldHeading == 90) {
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    BR.setPower(0);
                }
            }
        }
        //straight east
        else if (distanceY == 0 && distanceX > 0) {
            double goalFieldHeading = 0;
            while (currentFieldHeading != goalFieldHeading) {
                BL.setPower(.2);
                FL.setPower(.2);
                BR.setPower(-.2);
                FR.setPower(-.2);
                //update yaw and field heading
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                currentFieldHeading = yaw + startingHeading;
                //UPDATE currentFieldHeading AT END OF METHOD
                if (currentFieldHeading == 90) {
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    BR.setPower(0);
                }
            }
        }
        //straight south
        else if (distanceX == 0 && distanceY < 0) {
            double goalFieldHeading = 270;
            while (currentFieldHeading != goalFieldHeading) {
                BL.setPower(.2);
                FL.setPower(.2);
                BR.setPower(-.2);
                FR.setPower(-.2);
                //update yaw and field heading
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                currentFieldHeading = yaw + startingHeading;
                //UPDATE currentFieldHeading AT END OF METHOD
                if (currentFieldHeading == 90) {
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    BR.setPower(0);
                }
            }
        }
        //straight west
        else if (distanceY == 0 && distanceX < 0) {
            double goalFieldHeading = 180;
            while (currentFieldHeading != goalFieldHeading) {
                BL.setPower(.2);
                FL.setPower(.2);
                BR.setPower(-.2);
                FR.setPower(-.2);
                //update yaw and field heading
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                currentFieldHeading = yaw + startingHeading;
                //UPDATE currentFieldHeading AT END OF METHOD
                if (currentFieldHeading == 90) {
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    BR.setPower(0);
                }
            }
        }
        // no movement has no condition, nothing will happen if distance variables are 0

        //NOW WE NEED DISTANCE DRIVING FOR HYPOTENUSE. WE WILL DO THIS IN A PHYSICAL SPACE BECAUSE WE NEED TO CALCULATE TICKS PER INCH

        /* We need to:

        find hypotenuse with distanceX and Y (done at top of method)

        convert hypotenuse length to inches: 360 imaginary units (i units) = 12 feet = 144 inches (done below)

        convert inches to ticks (need a ratio for this) (ratio at top), needs verifiaction

        run the motor(s) with encoder using ticks to achieve distance
         */

        //calculate average so that it does each motor the right amount.
        //ticks dont scale at the same speed because to go soem diretions, it doesnt go forward, it actually goes backwards.
        double tickAverage = (BL.getCurrentPosition() + BR.getCurrentPosition()+ FR.getCurrentPosition()+ FL.getCurrentPosition()) / 4;
        double rawTickDistance = (hypotenuse / 2.5) * ticksPerInch; //2.5 = 360/144, so its inches * ticksPerInch
        double ticksToGo = tickAverage + rawTickDistance;
        //checks if goal has been met
        while (ticksToGo <= (tickAverage-2) || ticksToGo >= (tickAverage+2)) { //idk if getCurrentPosition will update automatically, different way of checking goal may be needed
            BL.setTargetPosition((int) ticksToGo);
            FL.setTargetPosition((int) ticksToGo);
            BR.setTargetPosition((int) ticksToGo);
            FR.setTargetPosition((int) ticksToGo);

            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BL.setPower(.5);
            FL.setPower(.5);
            BR.setPower(.5);
            FR.setPower(.5);
            if (ticksToGo > (tickAverage-2) && ticksToGo < (tickAverage+2)) {
                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
            }
        }



    }



    @Override
    public void runOpMode() {


        BL = hardwareMap.get(DcMotor.class, "motor_back_left");
        BR = hardwareMap.get(DcMotor.class, "motor_back_right");
        FL = hardwareMap.get(DcMotor.class, "motor_front_left");
        FR = hardwareMap.get(DcMotor.class, "motor_front_right");
        LFL = hardwareMap.get(DcMotor.class, "lift_front_left");
        LFR = hardwareMap.get(DcMotor.class, "lift_front_right");
        Push = hardwareMap.get(Servo.class, "pusher");
        Plane = hardwareMap.get(Servo.class, "plane");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(myIMUparameters);

        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        while (opModeInInit()) {
            //may need to loop to get starting heading, but this is a good start
            //this probably isnt allowed by rules
            /*
            telemetry.addData("", telemetryMessage); //must have at least 2 strings
            telemetry.update();
            if (gamepad1.a) {
                startingHeading = 90;
            } else if (gamepad1.b) {
                startingHeading = 180;
            } else if (gamepad1.x) {
                startingHeading = 0;
            } else if (gamepad1.y) {
                startingHeading = 270;
            } else if (gamepad1.dpad_up) {
                startingHeading = 45;
            } else if (gamepad1.dpad_down) {
                startingHeading = 225;
            } else if (gamepad1.dpad_left) {
                startingHeading = 135;
            } else if (gamepad1.dpad_right) {
                startingHeading = 315;
            }
            telemetry.addData("Starting heading set to: ", startingHeading);
            telemetry.update();

             */

        }
        
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //update all variables
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                currentFieldHeading = yaw+startingHeading;

                //april tag & camera method

                
            }
        }

    }


}