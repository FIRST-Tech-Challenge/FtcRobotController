package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Driving358 extends LinearOpMode {


    Hardware358 robot = new Hardware358();

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: REV Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     INCHES_FOR_RIGHT_ANGLE  = 4;

    static final double     LIFT_COUNTS_FULL_REVOLVE= 1440 /4;

    final double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)
    //  The GAIN constants set the relationship between the measured position error,
    //  and how much power is applied to the drive motors.  Drive = Error * Gain
    //  Make these values smaller for smoother control.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MM_PER_INCH = 25.40 ;   //  Metric conversion

    public void motorStop() {
        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);

        //robot.rotateLeft.setPower(0);
        //robot.rotateRight.setPower(0);
//        robot.carousel.setPower(0);
        reset();
    }
    public void reset(){
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //.m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   //     robot.rotateRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //     robot.rotateRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

   }
//
//    public void distanceMove(double distance, boolean relativeMove){
//        double reversePercent = 1; //speed percentage when going backward
//        double power = .1; //this is the power for the 2022 neverest20 motors
//
//        //measure and calculate the starting and goal distances based on relative and absolute movement
//        //RELATIVE (relative move = true): move 20 cm from where I am now (if you start 10 cm from the wall, you will move 20 cm to a total of 30 cm)
//        //ABSOLUTE (relative move = false): move until you are 20 cm from wall (if you start 10 cm from the wall, you will move 10cm, to a total of 20 cm)
//
//        
//        double goalDist = distance;
////        if(relativeMove) {
////            goalDist+=startingDist;
////        }
//
//        long currentTime = System.currentTimeMillis();
//        long timeOutTime = 7000L;
//        long totalTime = currentTime + timeOutTime;
//
//        double currentDist = 0; //robot.backDist.getDistance(DistanceUnit.CM);
//
//        while (currentDist<=goalDist){//until the current dist is greater than the goal
//            robot.lf.setPower(power);
//            robot.rf.setPower(power);
//            robot.rb.setPower(power);
//            robot.lb.setPower(power);
//        //    currentDist = robot.backDist.getDistance(DistanceUnit.CM);
//            telemetry.addData("Current Dist", currentDist);
//            telemetry.update();
//
////            if (totalTime < System.currentTimeMillis()){
////                return;
////            }
//        }
//        motorStop();
//        sleep(200);
//       // currentDist = robot.backDist.getDistance(DistanceUnit.CM);
//
//        if (currentDist> goalDist+2.0){//this may or may not be working, but the code is precise enough that right now, we don't have to worry XOXO Quyen Feb7, 2022
//            robot.lf.setPower(-power *reversePercent);
//            robot.rf.setPower(-power *reversePercent );
//            robot.lb.setPower(-power *reversePercent);
//            robot.rb.setPower(-power *reversePercent);
//        //    currentDist = robot.backDist.getDistance(DistanceUnit.CM);
//            telemetry.addData("Current Dist (reverse)", currentDist);
//            telemetry.update();
////            if (totalTime < System.currentTimeMillis()){
////                return;
////            }
//        }
//        motorStop();
//    }
//    public void strafeMove(double distance, boolean relativeMove){
//        double reversePercent = 1; //speed percentage when going backward
//        double power = .1; //this is the power for the 2022 neverest20 motors
//
//        //measure and calculate the starting and goal distances based on relative and absolute movement
//        //RELATIVE (relative move = true): move 20 cm from where I am now (if you start 10 cm from the wall, you will move 20 cm to a total of 30 cm)
//        //ABSOLUTE (relative move = false): move until you are 20 cm from wall (if you start 10 cm from the wall, you will move 10cm, to a total of 20 cm)
//
//        double startingDist = 0; //robot.backDist.getDistance(DistanceUnit.CM);
//        double goalDist = distance;
//        if(relativeMove) {
//            goalDist+=startingDist;
//        }
//
//        long currentTime = System.currentTimeMillis();
//        long timeOutTime = 7000L;
//        long totalTime = currentTime + timeOutTime;
//
//        double currentDist = robot.strafeDist.getDistance(DistanceUnit.CM);
//
//        while (currentDist<=goalDist){//until the current dist is greater than the goal
//            robot.lf.setPower(power);
//            robot.rf.setPower(-power);
//            robot.lb.setPower(-power);
//            robot.rb.setPower(power);
//            currentDist = robot.strafeDist.getDistance(DistanceUnit.CM);
//            telemetry.addData("Current Dist", currentDist);
//            telemetry.update();
//
////            if (totalTime < System.currentTimeMillis()){
////                return;
////            }
//        }
//        motorStop();
//        sleep(200);
//        currentDist = robot.strafeDist.getDistance(DistanceUnit.CM);
//
//        if (currentDist> goalDist+2.0){//this may or may not be working, but the code is precise enough that right now, we don't have to worry XOXO Quyen Feb7, 2022
//            robot.lf.setPower(-power *reversePercent);
//            robot.rf.setPower(power *reversePercent );
//            robot.lb.setPower(power *reversePercent);
//            robot.rb.setPower(-power *reversePercent);
//            currentDist = robot.strafeDist.getDistance(DistanceUnit.CM);
//            telemetry.addData("Current Dist (reverse)", currentDist);
//            telemetry.update();
////            if (totalTime < System.currentTimeMillis()){
////                return;
////            }
//        }
//        motorStop();
//    }
    public void move(double power, char direction, double distance){
        double ticks = COUNTS_PER_INCH * distance/3;
//        double ticks = 7.5* distance;
        switch(direction){
            case 'f':
                //to go forward

                //set target position
                robot.lf.setTargetPosition((int)ticks);
                robot.lb.setTargetPosition((int)(ticks));
                robot.rf.setTargetPosition((int)(ticks));
                robot.rb.setTargetPosition((int)(ticks));
                //set run to position
                robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.lf.setPower(power);
                robot.rf.setPower(power);
                robot.lb.setPower(power);
                robot.rb.setPower(power);

                while (robot.lf.isBusy() && robot.lb.isBusy() && robot.rf.isBusy() && robot.rb.isBusy())
                {

                }
                motorStop();
                robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;

            case 'b':
                //setting power of motors to go backward


                //set target position
                robot.lf.setTargetPosition((int) -ticks);
                robot.lb.setTargetPosition((int) -ticks);
                robot.rf.setTargetPosition((int) -ticks);
                robot.rb.setTargetPosition((int) -ticks);
                //set run to position
                robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.lf.setPower(-power);
                robot.rf.setPower(-power);
                robot.lb.setPower(-power);
                robot.rb.setPower(-power);

                while (robot.lf.isBusy() && robot.lb.isBusy() && robot.rf.isBusy() && robot.rb.isBusy())
                {
                    telemetry.clear();
                    telemetry.addData("Front Left Pos", robot.lf.getCurrentPosition());
                    telemetry.addData("Front Right Pos", robot.rf.getCurrentPosition());
                    telemetry.addData("Back Left Pos", robot.lb.getCurrentPosition());
                    telemetry.addData("Back Right (Mephistopheles) Pos", robot.rb.getCurrentPosition());
                    telemetry.update();

                }
                motorStop();
                robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;

            case 'r':
                //to strafe right


                //set target position
                robot.lf.setTargetPosition((int) ticks);
                robot.lb.setTargetPosition((int)-ticks);
                robot.rf.setTargetPosition((int)-ticks);
                robot.rb.setTargetPosition((int) ticks);
                //set run to position
                robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.lf.setPower(power);
                robot.rf.setPower(-power);
                robot.lb.setPower(-power);
                robot.rb.setPower(power);

                while (robot.lf.isBusy() && robot.lb.isBusy() && robot.rf.isBusy() && robot.rb.isBusy())
                {

                }
                motorStop();
                robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            case 'l' :
                // to strafe left

                //set target position
                robot.lf.setTargetPosition((int)-ticks);
                robot.lb.setTargetPosition((int)ticks);
                robot.rf.setTargetPosition((int)ticks);
                robot.rb.setTargetPosition((int)-ticks);
                //set run to position
                robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.lf.setPower(-power);
                robot.rf.setPower(power);
                robot.lb.setPower(power);
                robot.rb.setPower(-power);

                while (robot.lf.isBusy() && robot.lb.isBusy() && robot.rf.isBusy() && robot.rb.isBusy())
                {

                }
                motorStop();
                robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;

            default:
                motorStop();
        }
    }

    public void rotate(double power, char direction, double angle) {
        double ticks = COUNTS_PER_INCH * angle / 90 * INCHES_FOR_RIGHT_ANGLE;
//        double ticks = 7.5* distance;
        switch(direction){
            case 'r':
                //to turn clockwise

                robot.lf.setTargetPosition((int)ticks);
                robot.lb.setTargetPosition((int)ticks);
                robot.rf.setTargetPosition((int)-ticks);
                robot.rb.setTargetPosition((int)-ticks);
                //set run to position
                robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.lf.setPower(power);
                robot.rf.setPower(-power);
                robot.lb.setPower(power);
                robot.rb.setPower(-power);

                while (robot.lf.isBusy() && robot.lb.isBusy() && robot.rf.isBusy() && robot.rb.isBusy())
                {

                }
                motorStop();
                robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            case 'l':
                // to turn counter clockwise

                robot.lf.setTargetPosition((int)-ticks);
                robot.lb.setTargetPosition((int) -ticks);
                robot.rf.setTargetPosition((int)ticks);
                robot.rb.setTargetPosition((int) ticks);
                //set run to position
                robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.lf.setPower(-power);
                robot.rf.setPower(-power);
                robot.lb.setPower(power);
                robot.rb.setPower(power);

                while (robot.lf.isBusy() && robot.lb.isBusy() && robot.rf.isBusy() && robot.rb.isBusy())
                {
                    telemetry.clear();
                    telemetry.addData("Front Left Pos", robot.lf.getCurrentPosition());
                    telemetry.addData("Front Right Pos", robot.rf.getCurrentPosition());
                    telemetry.addData("Back Left Pos", robot.lb.getCurrentPosition());
                    telemetry.addData("Back Right (Mephistopheles) Pos", robot.rb.getCurrentPosition());
                    telemetry.update();
                }
                motorStop();
                robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            default:
                motorStop();
        }
    }
    public void diagonal(double power, char direction, long distance){
        double ticks = 1120/7.5 * distance;
        switch(direction) {
            case '1':
                //forward right

                //set target position

                robot.lf.setTargetPosition((int) (ticks));
                robot.lb.setTargetPosition(0);
                robot.rf.setTargetPosition(0);
                robot.rb.setTargetPosition((int) ticks);
                //set run to position
                robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.lf.setPower(power);
                robot.rf.setPower(0);
                robot.lb.setPower(0);
                robot.rb.setPower(power);

                while (robot.lf.isBusy() && robot.lb.isBusy() && robot.rf.isBusy() && robot.rb.isBusy()) {

                }
                motorStop();
                robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;

            case '2':
                //forward left

                //set target position

                robot.lf.setTargetPosition(0);
                robot.lb.setTargetPosition((int) ticks);
                robot.rf.setTargetPosition((int) ticks);
                robot.rb.setTargetPosition(0);
                //set run to position
                robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.lf.setPower(0);
                robot.rf.setPower(power);
                robot.lb.setPower(power);
                robot.rb.setPower(0);

                while (robot.lf.isBusy() && robot.lb.isBusy() && robot.rf.isBusy() && robot.rb.isBusy()) {

                }
                motorStop();
                robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                break;
            case '3':
                // go back right

                robot.lf.setTargetPosition(0);
                robot.lb.setTargetPosition((int) -ticks);
                robot.rf.setTargetPosition((int) -ticks);
                robot.rb.setTargetPosition(0);
                //set run to position
                robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.lf.setPower(0);
                robot.rf.setPower(-power);
                robot.lb.setPower(-power);
                robot.rb.setPower(0);

                while (robot.lf.isBusy() && robot.lb.isBusy() && robot.rf.isBusy() && robot.rb.isBusy()) {

                }
                motorStop();
                robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                break;

            case '4':
//back left
                robot.lf.setTargetPosition((int) -ticks);
                robot.lb.setTargetPosition(0);
                robot.rf.setTargetPosition(0);
                robot.rb.setTargetPosition((int) -ticks);
                //set run to position
                robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.lf.setPower(-power);
                robot.rf.setPower(0);
                robot.lb.setPower(0);
                robot.rb.setPower(-power);

                while (robot.lf.isBusy() && robot.lb.isBusy() && robot.rf.isBusy() && robot.rb.isBusy()) {

                }
                motorStop();
                robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                break;
            default:
                motorStop();

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

//    public void lift (double power, int level, long moveTime){
//        robot.rotateLeft.setPower(.05*power);
//        robot.rotateRight.setPower(.05*power);
//
//        long currentTime = System.currentTimeMillis();
//        boolean atPosition = false;
//        while (currentTime + moveTime > System.currentTimeMillis() && !atPosition){
//            telemetry.addData("Target Time", currentTime+moveTime);
//            telemetry.addData("Current time", System.currentTimeMillis());
//            telemetry.update();
//            if (robot.magStopBottom.getValue()==1.0 && level !=1){
//                robot.rotateRight.setPower(-.1);
//                robot.rotateLeft.setPower(-.1);
//                break;
//            }
//            switch (level){
//                case 1:
//                    if (robot.magStopBottom.getValue()==1.0){
//                        robot.rotateRight.setPower(-.37);
//                        robot.rotateLeft.setPower(-.37);
//                        telemetry.addData("Stopped", 1);
//                        telemetry.update();
//                        sleep(200);
//                        motorStop();
//                        robot.rotateRight.setPower(-.05);
//                        robot.rotateRight.setPower(-.05);
//                        sleep(300);
//                        motorStop();
//                        break;
//                    }
//                    break;
//                case 2:
//                    if (robot.magStopMid.getValue()==1.0){
//                        robot.rotateRight.setPower(-.37);
//                        robot.rotateLeft.setPower(-.37);
//                        telemetry.addData("Stopped", 2);
//                        telemetry.update();
//                        sleep(200);
//                        atPosition = true;
//                        break;
//                    }
//                    break;
//                case 3:
//                    if (robot.magStopTop.getValue()==1.0){
//                        sleep(150);
//                        robot.rotateRight.setPower(-.1);
//                        robot.rotateLeft.setPower(-.1);
//                        sleep(50);
//                        atPosition = true;
//                        break;
//                    }
//                    break;
//                default:
//                    break;
//            }
//       }
//
//        motorStop();
//
//
//        //Theoretical way to get it to run on encoders. doesn't work for now.
////        double ticks = LIFT_COUNTS_FULL_REVOLVE * anglePercent /20;
////
////        robot.rotateLeft.setTargetPosition((int)ticks);
////        robot.rotateRight.setTargetPosition((int)ticks);
////
////        robot.rotateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.rotateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////        robot.rotateLeft.setPower(.02);
////        robot.rotateRight.setPower(.02);
////
////
////        while (robot.rotateLeft.isBusy()){
////            telemetry.addData("Target:", (int)ticks);
////            telemetry.addData("Current", robot.rotateLeft.getCurrentPosition());
////            telemetry.update();
////
////        }
////        telemetry.addData("OUT", "YUP");
////        telemetry.update();
////        motorStop();
//    }

//    public void levelLift (char clawTarget){
//        double lowPosition = 90;//target positions, in mm
//        double midPosition = 108;
//        double highPosition = 243;
//
//        double precisePower = .03;//powers of the motor in different modes
//        double roughPower = .05;
//        double correctionSpeed = .1;
//
//        int waitTime = 100;
//        int distanceCutoff = 2000;
//
//        double targetPosition;
//
//        switch (clawTarget){//sets the target based on the user's input
//            case 'l':
//            case 'b':
//                targetPosition = lowPosition;
//                break;
//            case 'm':
//                targetPosition = midPosition;
//                break;
//            case 't':
//            case 'h':
//                targetPosition = highPosition;
//                break;
//            default:
//                targetPosition = 3000;//this is a test number. Should read very high if not pointing at anything.
//        }
//
//        double currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//        motorStop();
//
//        while (currentPosition - 500 > targetPosition){//if the current position is 50 cm more than the target position, it moves very speedily to get there
//            robot.rotateLeft.setPower(roughPower);//it should only be that large if the claw is in the upward position.
//            robot.rotateRight.setPower(roughPower);
//
//            currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//            telemetry.addData("Mode:", "Rough Movement");
//            telemetry.addData("Target Position:", targetPosition);
//            telemetry.addData("Current Position:", currentPosition);
//            telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//            telemetry.update();
//        }
//
//        motorStop();
//
//        sleep(waitTime);
//        telemetry.addData("Mode:", "Waiting for Fine");
//        telemetry.addData("Target Position:", targetPosition);
//        telemetry.addData("Current Position:", currentPosition);
//        telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//        telemetry.update();
//        sleep(waitTime);
//        currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//
//        while (currentPosition > targetPosition){//once it gets closer, it slows down
//            robot.rotateLeft.setPower(precisePower);
//            robot.rotateRight.setPower(precisePower);
//
//            currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//            telemetry.addData("Mode:", "Fine Movement");
//            telemetry.addData("Target Position:", targetPosition);
//            telemetry.addData("Current Position:", currentPosition);
//            telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//            telemetry.update();
//        }
//        motorStop();
//        sleep(waitTime);
//        telemetry.addData("Mode:", "Waiting for Correct");
//        telemetry.addData("Target Position:", targetPosition);
//        telemetry.addData("Current Position:", currentPosition);
//        telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//        telemetry.update();
//        sleep(waitTime);
//        currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//        int correctionError = 10;
//        if (clawTarget == 't'||clawTarget=='h'){
//            correctionError = 0;
//        }
//
//        while (currentPosition + correctionError < targetPosition){//once it is close, it goes back up until it is within 1 cm.
//            robot.rotateLeft.setPower(-correctionSpeed);
//            robot.rotateRight.setPower(-correctionSpeed);
//
//            currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//            telemetry.addData("Mode:", "Correction Movement");
//            telemetry.addData("Target Position:", targetPosition);
//            telemetry.addData("Current Position:", currentPosition);
//            telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//            telemetry.update();
//
//            if((int)currentPosition > distanceCutoff){//if it gets super high, it will (hopefully) stop.
//                break;
//            }
//        }
//        motorStop();
//        sleep(waitTime);
//        currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//        telemetry.addData("Mode:", "Done");
//        telemetry.addData("Target Position:", targetPosition);
//        telemetry.addData("Current Position:", currentPosition);
//        telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//        telemetry.update();
//        sleep(waitTime);
//    }
}
