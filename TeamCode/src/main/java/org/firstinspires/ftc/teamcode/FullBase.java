package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UltimateGoalComponents.*;

public class FullBase extends robotBase {
    public Drivetrain drivetrain;
    public Hopper hopper;
    public Shooter shooter;
    /*
    public WobbleArm wobbleArm;
    public Intake intake;
    */
    private RobotComponent[] components = new RobotComponent[5];

    public FullBase(Telemetry telemetry, LinearOpMode opMode, HardwareMap hardwaremap) {
        super(telemetry, opMode, hardwaremap);
    }



    @Override
    public void init() {
        //create drivetrain
        telemetry.addLine("Drivetrain about to init");
        drivetrain = new Drivetrain(this);
        telemetry.addLine("drive inited");
        components[0] = drivetrain;

        //create hopper
        hopper = new Hopper(this);
        hopper.setHopperPosition(Hopper.Position.INIT_POSITION);
        hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        components[1] = hopper;

        //create shooter
        shooter = new Shooter(this);
        components[2] = shooter;
/*

        //create wobbleArm
        wobbleArm = new WobbleArm(this);
        components[3] = wobbleArm;

        //create intake
        intake = new Intake (this);
        components[4] = intake;
    */
    }

    @Override
    public void stop() {
        for( int i = 0; i<=1; i++){
            components[i].stop();
        }
    }
}
