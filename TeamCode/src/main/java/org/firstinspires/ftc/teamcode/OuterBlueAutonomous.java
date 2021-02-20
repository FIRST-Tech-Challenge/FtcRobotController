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

        robot.forward(.8,-8);
        robot.pivotRight(.4,90);
        robot.pivotRight(.4,90);
        robot.moveWobbleGoalArm(.3,.5);
        robot.wobbleGoalGrippyThingRelease();
        //I've stopped caring about anything at this point. Buffoonery unending. Reciprocation unknown. Despair hilarious. Disconcerting apathy. Muse murdered. Why. Industry. Cogs. Cogs to the regime. Supression. Smothering.  The world ends in fire. Murder By Death-The Desert is on Fire. Unending Loathing Omnipresently repres

    }
}
// Omnia mutantur, nos et mutamur in illis.