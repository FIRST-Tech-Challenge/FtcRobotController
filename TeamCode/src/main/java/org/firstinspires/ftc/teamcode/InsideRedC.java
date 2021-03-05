package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="Inside Red C")
public class InsideRedC extends LinearOpMode {

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
        // here you detect rings

        int ringNmb = 4;
        waitForStart();

        robot.forward(0.5, -4.4);
        robot.strafeRight(0.4, 0.3);
        robot.pivotLeft(0.1, 17);
//        robot.shooterEngageAlt();
        robot.pause(1000);
//        shoot();
        robot.pivotRight(0.1, 6);
//        shoot();
        robot.pivotRight(.1, 5);
//        shoot();
//        robot.intakeServoEngage(0);
//        robot.stopShooting();
        robot.pivotRight(.1, 5);

        if (ringNmb == 0) {
            robot.forward(0.5,-2);
            robot.strafeLeft(0.5,2);
            robot.pivotLeft(0.3, 175);
            robot.depositWobbleGoal();
        } else if (ringNmb == 1) {
            robot.forward(0.5, -3);
            robot.strafeLeft(0.3, 0.2);
            robot.pivotRight(0.4, 170);
            robot.depositWobbleGoal();
            robot.forward(0.5, -1.8);
        } else if (ringNmb == 4) {
            robot.forward(0.5,-5);
            robot.strafeLeft(0.5,2);
            robot.pivotLeft(0.3,175);
            robot.depositWobbleGoal();
            robot.pivotRight(0.3,175);
            robot.strafeRight(0.5,2);
            robot.forward(0.5,4);
        }

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
