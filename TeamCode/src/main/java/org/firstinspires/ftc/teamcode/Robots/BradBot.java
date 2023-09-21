package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.FutureComponents.Extendo;
import org.firstinspires.ftc.teamcode.Components.FutureComponents.FlippingIntake;
import org.firstinspires.ftc.teamcode.Components.Hopper;
import org.firstinspires.ftc.teamcode.Components.Intake;
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
        lift = new Lift();
        roadrun = new SampleMecanumDrive(p_op.hardwareMap);
//        ultras = new Ultrasonics();
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
