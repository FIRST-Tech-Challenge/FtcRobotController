package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.VariablesBase;


//Blue Autonomous: Delivers Duck and Parks in WH/
//Starting Position: Back facing Carousel (10 degrees from wall)

@Autonomous(name= "Test")
public class Test extends LinearOpMode{

    MainBase base = new MainBase();
    VariablesBase var = new VariablesBase();


    @Override
    public void runOpMode() throws InterruptedException {


        base.init(hardwareMap, this);

        base.frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("God Speed");
        telemetry.update();

        waitForStart();


        base.gyroStrafe(60,0,var.DRIVE_SPEED07,this);
        base.gyroHold(.2, 0,3,this);


    }
}
