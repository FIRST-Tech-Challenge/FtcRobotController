package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
        robot= new RobotClass(hardwareMap, telemetry, this);

        robot.wobbleGoalGrippyThingGrab();
        // here you detect rings, no rings=0 and so on

        int ringNmb = 1;
        waitForStart();

        robot.forward(0.5, -4.4);
        robot.strafeRight(0.4, 0.3);
        robot.pivotLeft(0.1, 17);
        robot.shooterEngageAlt();
        robot.pause(1000);

        shoot();
        robot.pivotRight(0.1, 6);

        shoot();
        robot.pivotRight(.1, 5);
        shoot();
        robot.intakeServoEngage(0);
        robot.stopShooting();
        // this is where the number of rings matter
        robot.pivotRight(.1, 5);
        robot.forward(0.5, -3);
        robot.strafeLeft(0.3, 0.2);
        robot.pivotRight(0.4, 170);
        //robot.pivotRight(0.1, 7);
        robot.depositWobbleGoal();
        robot.forward(0.5, -1.8);

//        robot.forward(0.5,-2);
//        if (ringNmb == 0) {
//            //strafe right
//            robot.strafeRight(0.5, 2);
//        } else if (ringNmb == 1) {
//            robot.forward(0.5,-3);
//            robot.moveWobbleGoalArm(0.7,0.4);
//            robot.wobbleGoalGrippyThingRelease();
//            robot.forward(0.5,2);
//        } else if (ringNmb == 3) {
//            robot.forward(0.5,-4);
        //forward+ strafe right+ wobble goal
//        }

    }
    protected void shoot(){
        robot.shooterServo1(.8);
        robot.shooterServo2(.8);
        robot.pause(200);
        robot.intakeServoEngage(.9);
        robot.pause(800);
        robot.shooterServo2(0);
        robot.shooterServo1(0);
    }
//    protected void shootLast(){
//        robot.shooterServo1(.8);
//        robot.shooterServo2(.8);
//        robot.pause(1200);
//        robot.shooterServo2(0);
//        robot.shooterServo1(0);
//    }
}

