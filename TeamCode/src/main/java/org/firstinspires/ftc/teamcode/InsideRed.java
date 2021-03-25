package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Date;

@Autonomous (name="Inside Red")
public class InsideRed extends LinearOpMode {

    DcMotor wobbleGoalExtendMotor = null;
    DcMotor wobbleGoalRaiseMotor = null;
    Servo wobbleGoalGrippyThing = null;
    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {

        wobbleGoalExtendMotor = hardwareMap.dcMotor.get("wobbleExtendo");
        wobbleGoalRaiseMotor = hardwareMap.dcMotor.get("wobbleLift");
        wobbleGoalGrippyThing = hardwareMap.servo.get("wobbleGrip");
        robot= new RobotClass(hardwareMap, telemetry, this, "red");

        robot.wobbleGoalGrippyThingGrab();
        // here you detect rings
        robot.openCVInnitShenanigans();

        RobotClass.RingPosition ringNmb = null;
        waitForStart();
        robot.forward(0.1, -0.3);
        robot.strafeLeft(0.4,1.7);

        long startTime = new Date().getTime();
        long time = 0;
        while (time < 500 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
             ringNmb = robot.analyze();

            telemetry.addData("Position", ringNmb);
            telemetry.update();
        }
        robot.strafeRight(0.5,2.3);
        robot.forward(0.6, -4.1);
        //robot.strafeRight(0.4, 0.3);
        robot.pivotLeft(0.1, 17);
//        robot.shooterEngageAlt();
        robot.pause(500);
//        shoot();
        robot.pivotRight(0.1, 6);
//        shoot();
        robot.pivotRight(.1, 5);
//        shoot();
        robot.intakeServoEngage(0);
        robot.pause(200);
//        robot.stopShooting();
        robot.pivotRight(.1, 6);

        if (ringNmb == RobotClass.RingPosition.NONE) {
            robot.forward(0.5,-1.5);
            robot.strafeLeft(0.5,2);
            robot.pivotLeft(0.3, 170);
            robot.depositWobbleGoal();
            robot.strafeLeft(0.4, 2);
        } else if (ringNmb == RobotClass.RingPosition.ONE) {
            robot.forward(0.5, -3);
            robot.strafeLeft(0.3, 0.2);
            robot.pivotRight(0.4, 170);
            robot.depositWobbleGoal();
            robot.forward(0.5, -1.8);
        } else if (ringNmb == RobotClass.RingPosition.FOUR) {
            robot.forward(0.8,-4.5);
            robot.strafeLeft(0.8,3.2);
            robot.pivotLeft(0.4,170);
            robot.depositWobbleGoal();
           // robot.pivotRight(0.3,175);
            robot.strafeLeft(0.8,1);
            robot.forward(0.8,-2.8);
        }

    }
//    protected void shoot(){
//        robot.shooterServo1(.8);
//        robot.shooterServo2(.8);
//        robot.pause(200);
//        robot.intakeServoEngage(.9);
//        robot.pause(800);
//        robot.shooterServo2(0);
//        robot.shooterServo1(0);
    }
//    protected void shootLast(){
//        robot.shooterServo1(.8);
//        robot.shooterServo2(.8);
//        robot.pause(1200);
//        robot.shooterServo2(0);
//        robot.shooterServo1(0);


