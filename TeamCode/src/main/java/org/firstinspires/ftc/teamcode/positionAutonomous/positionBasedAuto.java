package org.firstinspires.ftc.teamcode.positionAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Position Based Auto", group = "Autonomous")
public class positionBasedAuto extends LinearOpMode {
    // Arm components
    private DcMotor mantis; //EXP PRT 0
    private DcMotor lift; //EXP PRT 2
    private DcMotor hopper; //EXP PRT 3

    // Wrist and door
    private CRServo bottomGrabber; //EXP PRT 0
    private CRServo topGrabber; //EXP PRT 1
    private Servo door; //EXP PRT 2

    // Wheel components
    private DcMotor frontLeft; //CTRL PRT 0
    private DcMotor frontRight; //CTRL PRT 1
    private DcMotor backLeft; //CTRL PRT 2
    private DcMotor backRight; //CTRL PRT 3

    private final String[] puns = {
            "A robot didn’t want to have his photo taken. When he was asked why, he replied: Because I’m a photo-resistor!",
            "A robot gets arrested. He’s charged with battery.",
            "A robot man walks into a robot restaurant. A robot waiter approaches and asks him for his robot order.",
            "A robot musician’s collection of instruments will never be complete. They can never get any organs.",
            "A robot walks into a bar and says he needs to loosen up. So the bartender serves him a screwdriver.",
            "A robot walks into a bar. The bartender asks, 'What’ll ya have?' The robot says, 'Well, it’s been a long day and I need to loosen up. How about a screwdriver?'",
            "Did you hear about the writing robot who combined all the books ever written into one big novel? It’s a long story.",
            "Does R2D2 have any brothers? No. Only transisters.",
            "Hey, did you hear the story about the headless robot? According to reports, he completely lost his mind!",
            "How are A.I. blogs similar to philosophy majors? That’s easy… they’re both always trying to explain what ‘deep learning’ is!",
            "How did the robot get across the river? In a ro-boat.",
            "How did the robot’s teacher mark his book? With robo-ticks.",
            "How do you know when you’re in love with a robot? You feel a little spark.",
            "How do you reboot a robot? You kick it in its robutt.",
            "How do you use a remote control to calm down a robot dog? Press the paws button.",
            "How long is the robot alphabet? There are just two numbers – 0 and 1!",
            "How many robots does it take to screw in a light bulb? Three — one to hold the bulb, and two to turn the ladder!",
            "I bought one of those early 2000s robot dogs but have nowhere to charge it; I can’t find a place to pug it in.",
            "I finally fulfilled my dream to become a half-cyborg! It did cost me an arm and a leg, though.",
            "I got a new robot dog last week. Its name is Dogmatic.",
            "I invented a surgical robot. So far it only operates on batteries.",
            "I just got a wireless robot the other day. You could say that our relationship comes with no strings attached.",
            "I was bored, so I made a robot that distributes herbs. It helped pass the thyme.",
            "I’m not saying all factory workers are robots… All I’m saying is when they get to work they’ve returned to their factory setting.",
            "I’m starting to make a robot that has really high words per minute count. He’s a pro-to-type.",
            "If a Norwegian robot analyzed a bird, then it… Scandinavian.",
            "In a robot-only disco, one of the dancers suddenly shuts down. The doctor arrives and after a quick inspection he calms the crowd: 'Don’t worry, he just got disco-nnected.'",
            "Inventor: 'Hey, will you give me a hand?' Robot: Detaches hand, hands it to the inventor.",
            "Judge: 'So, Mr. Robot. Your neighbor accused you of stealing their electricity to power yourself. How do you plea?' Robot, the defendant: 'Guilty as charged.'",
            "My wife told me robots don’t wash themselves. So I put one in the bath and said 'That’ll shower.'",
            "Scientists have discovered a planet populated entirely by robots. They call it Mars.",
            "A robot orders a robot steak. The robot waiter asks them how they want their robot steak prepared. The robot replies, 'Weld on.'",
            "There was a giant, steel, robot who had one job, protect the city. One day when it was raining some of the screws got rusty and fell off causing one of the legs to fall off entirely. When the leg fell off it crushed the city that it was meant to protect. Oh, the iron knee!"
    };

    //TIME
    private ElapsedTime totalGameTime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    private double timeToRotate360 = 3.65;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setWheelDirection();
        setArmDirection();
        stopAndResetWheelEncoders();
        stopAndResetArmEncoders();

        waitForStart();
        while (opModeIsActive()) {

        }
    }

    //Initializes the components
    private void initialize() {
        //Wheels
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //Arms
        mantis = hardwareMap.get(DcMotor.class, "mantis");
        lift = hardwareMap.get(DcMotor.class, "lift");
        hopper = hardwareMap.get(DcMotor.class, "hopper");

        //Claws
        bottomGrabber = hardwareMap.get(CRServo.class, "bottomGrabber");
        topGrabber = hardwareMap.get(CRServo.class, "topGrabber");
        door = hardwareMap.get(Servo.class, "door");
    }

    //Sets the directions of each component
    private void setWheelDirection() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    private void setArmDirection() {
        lift.setDirection(DcMotor.Direction.REVERSE);
        mantis.setDirection(DcMotor.Direction.REVERSE);
        hopper.setDirection(DcMotor.Direction.FORWARD);
    }

    //Resets the encoders for each component
    private void stopAndResetWheelEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void stopAndResetArmEncoders() {
        mantis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Sets the positions of each component
    private void setWheelsPosition(int flTargetEncoderCount, int frTargetEncoderCount, int blTargetEncoderCount, int brTargetEncoderCount) {
        frontLeft.setTargetPosition(flTargetEncoderCount);
        frontRight.setTargetPosition(frTargetEncoderCount);
        backLeft.setTargetPosition(blTargetEncoderCount);
        backRight.setTargetPosition(brTargetEncoderCount);
    }
    private void setArmPosition(String armType, int targetEncoderCount) {
        switch (armType) {
            case "MANTIS":
                mantis.setTargetPosition(targetEncoderCount);
                break;
            case "LIFT":
                lift.setTargetPosition(targetEncoderCount);
                break;
            case "HOPPER":
                hopper.setTargetPosition(targetEncoderCount);
                break;
        }
    }

    //Sets the run mode to run to position for each component
    private void runToWheelsPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    private void runToArmPosition(String armType) {
        switch (armType) {
            case "MANTIS":
                mantis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case "LIFT":
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case "HOPPER":
                hopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
        }
    }

    //Set the components speed
    private void setWheelSpeed(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        frontLeft.setPower(flSpeed);
        frontRight.setPower(frSpeed);
        backLeft.setPower(blSpeed);
        backRight.setPower(brSpeed);
    }

    private void setArmSpeed(String armType, double speed) {
        switch (armType) {
            case "MANTIS":
                mantis.setPower(speed);
                break;
            case "LIFT":
                lift.setPower(speed);
                break;
            case "HOPPER":
                hopper.setPower(speed);
                break;
        }
    }

    //Brake for the motors
    private void wheelBrake() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    private void armBrake(String armType) {
        switch (armType) {
            case "MANTIS":
                mantis.setPower(0.0);
                break;
            case "LIFT":
                lift.setPower(0.0);
                break;
            case "HOPPER":
                hopper.setPower(0.0);
                break;
        }
    }

    private void clawBrake(String clawType) {
        switch (clawType) {
            case "BOTTOM_GRABBER":
                bottomGrabber.setPower(0.0);
                break;
            case "TOP_GRABBER":
                topGrabber.setPower(0.0);
                break;
            case "DOOR":
                door.setPosition(door.getPosition());
                break;
        }
    }

    //Utalizes the position and speed functions to make a base for all directions
    private void baseWheelMovement(double flSpeed, double frSpeed, double blSpeed, double brSpeed,
                                   int flPosition, int frPosition, int blPosition, int brPosition) {
        setWheelsPosition(flPosition, frPosition, blPosition, brPosition);
        runToWheelsPosition();
        setWheelSpeed(flSpeed, frSpeed, blSpeed, brSpeed);
        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

        }
        wheelBrake();
        stopAndResetWheelEncoders();
    }
    private void setArmMovement(String armType, double speed, int position){
        setArmPosition(armType, position);
        runToArmPosition(armType);
        setArmSpeed(armType, speed);
        while (opModeIsActive() && (mantis.isBusy() || lift.isBusy() || hopper.isBusy())) {

        }
        armBrake(armType);
        stopAndResetArmEncoders();
    }
    private void setClawPosition(String clawType, double position) {
        switch (clawType) {
            case "BOTTOM_GRABBER":
                bottomGrabber.setPower(position);
                break;
            case "TOP_GRABBER":
                topGrabber.setPower(position);
                break;
            case "DOOR":
                door.setPosition(position);
                break;

        }
    }

    //Movement for the wheels, arms, and claws
    private void moveWheels(String direction, double speed, int position) {
        switch (direction) {
            case "FORWARD":
                baseWheelMovement(speed, speed, speed, speed, position, position, position, position);
                break;
            case "BACKWARD":
                baseWheelMovement(-speed, -speed, -speed, -speed, -position, -position, -position, -position);
                break;
            case "TURN_RIGHT":
                baseWheelMovement(speed, -speed, speed, -speed, position, -position, position, -position);
                break;
            case "TURN_LEFT":
                baseWheelMovement(-speed, speed, -speed, speed, -position, position, -position, position);
                break;
            case "STOP":
                wheelBrake();
                break;
        }
    }
    private void moveArm(String armType, double speed) {

    }
}
