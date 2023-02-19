package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.VariablesBase;


//Blue Autonomous: Delivers Duck and Parks in WH/
//Starting Position: Back facing Carousel (10 degrees from wall)

@Autonomous(name= "old")
public class old extends LinearOpMode{

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



        while(base.navx_device.isCalibrating()){
            telemetry.addLine("navx start");
            telemetry.update();
        }

        //reset navx yaw
        base.navx_device.zeroYaw();


        telemetry.addLine("GOD SPEED");
        telemetry.addLine("navx calibrated");
        telemetry.addData("compass heading:", base.navx_device.getCompassHeading());
        telemetry.addData("yaw", base.navx_device.getYaw());
        telemetry.update();




        waitForStart();


        base.MRgyroDrive(.5,30,30,30,30,0,this);
    }
}
