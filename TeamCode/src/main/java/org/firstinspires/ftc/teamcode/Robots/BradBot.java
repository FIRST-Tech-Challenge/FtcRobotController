package org.firstinspires.ftc.teamcode.Robots;

import static org.apache.commons.math3.util.FastMath.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.FutureComponents.Extendo;
import org.firstinspires.ftc.teamcode.Components.FutureComponents.FlippingIntake;
import org.firstinspires.ftc.teamcode.Components.Hopper;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Launcher;
import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Components.Ultrasonics;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/**
 * Warren
 * Robot class to contain all the season's functions
 */
public class BradBot extends BasicRobot{
    Arm arm;
    CVMaster cv;
    Intake intake;
    Hopper hopper;
    Launcher launcher;
    Lift lift;
    SampleMecanumDrive roadrun;
    Ultrasonics ultras;

    /**
     * Instatiates all the hardware and sets up initial states of some software
     * @param p_op opMode
     * @param p_is_Teleop is the program a teleop program
     */
    public BradBot(LinearOpMode p_op, boolean p_is_Teleop){
        super(p_op,p_is_Teleop);
        arm = new Arm();
        cv = new CVMaster();
        intake = new Intake();
        hopper = new Hopper();
        launcher = new Launcher();
        lift = new Lift();
        roadrun = new SampleMecanumDrive(p_op.hardwareMap);
//        ultras = new Ultrasonics();
    }
    public void teleOp(){
        boolean isA = gampad.readGamepad(op.gamepad1.a, "gamepad1_a", "toggleArm");
        boolean rightBumper = gampad.readGamepad(op.gamepad1.right_bumper, "gamepad1_right_bumper", "startIntake");
        boolean leftBumper = gampad.readGamepad(op.gamepad1.left_bumper, "gamepad1_left_bumper", "reverseIntake");
        boolean isB = gampad.readGamepad(op.gamepad1.b,"gamepad1_b", "shoot");
        boolean up = gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "lift Up");
        boolean down = gampad.readGamepad(op.gamepad1.dpad_down, "gamepad1_dpad_down", "lift down");
        float manualUp = op.gamepad1.right_trigger;
        float manualDown = op.gamepad1.left_trigger;
        if(isA){
//            arm.flip();
        }
        if(rightBumper){
            intake.intake();
        }
        if(leftBumper){
            intake.reverseIntake();
        }
        if(isB){
            launcher.shoot();
        }
        if(up){
//            lift.iterateUp();
        }
        if(down){
//            lift.iterateDown();
        }
        if(abs(manualUp-manualDown)>0.05){
//            lift.setManual(manualUp-manualDown);
        }
        roadrun.setWeightedDrivePower(new Pose2d(op.gamepad1.left_stick_y
                , op.gamepad1.left_stick_x
                , op.gamepad1.right_stick_x));
        update();
    }
    public void update(){
        super.update();
        arm.update();
        cv.update();
        intake.update();
        hopper.update();
        lift.update();
        roadrun.update();
//        ultras.update();
    }
}
