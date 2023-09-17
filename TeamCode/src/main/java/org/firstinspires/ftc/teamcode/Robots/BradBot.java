package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Extendo;
import org.firstinspires.ftc.teamcode.Components.FlippingIntake;
import org.firstinspires.ftc.teamcode.Components.Hopper;
import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Components.Ultrasonics;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/**
 * Warren
 */
public class BradBot extends BasicRobot{
    Arm arm;
    CVMaster cv;
    Extendo extendo;
    FlippingIntake intake;
    Hopper hopper;
    Lift lift;
    SampleMecanumDrive roadrun;
    Ultrasonics ultras;
    public BradBot(LinearOpMode p_op, boolean p_is_Teleop){
        super(p_op,p_is_Teleop);
        arm = new Arm();
        cv = new CVMaster();
        extendo = new Extendo();
        intake = new FlippingIntake();
        hopper = new Hopper();
        lift = new Lift();
        roadrun = new SampleMecanumDrive(p_op.hardwareMap);
        ultras = new Ultrasonics();
    }
    public void update(){
        super.update();
        arm.update();
        cv.update();
        extendo.update();
        intake.update();
        hopper.update();
        lift.update();
        roadrun.update();
        ultras.update();
    }
}
