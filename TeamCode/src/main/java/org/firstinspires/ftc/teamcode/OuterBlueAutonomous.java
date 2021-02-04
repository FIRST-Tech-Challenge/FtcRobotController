package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "OuterBlueAutonomous")
public class OuterBlueAutonomous extends LinearOpMode{

    RobotClass robot;


    @Override
    public void runOpMode() {

        robot= new RobotClass(hardwareMap, telemetry, this);

        robot.wobbleGoalGrippyThingGrab();
        waitForStart();

        /*
        Forward until blue
        Deposit wobble goal
        Backwards to white
        Shoot much, a lot, good.
         */

        robot.forward(.8,-8);
        robot.turnRight(.4,180);
        robot.moveWobbleGoalArm(.3,.5);
        robot.wobbleGoalGrippyThingRelease();//semi-colander

    }
}
