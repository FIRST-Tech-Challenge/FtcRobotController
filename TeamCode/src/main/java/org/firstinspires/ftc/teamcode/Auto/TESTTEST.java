package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Base.MainBaseRight;
import org.firstinspires.ftc.teamcode.Base.VariablesBase;
import org.firstinspires.ftc.teamcode.Auto.VisionLeft;
import org.firstinspires.ftc.teamcode.Auto.VisionRight;



//Blue Autonomous: Delivers Duck and Parks in WH/
//Starting Position: Back facing Carousel (10 degrees from wall)

@Autonomous(name= "TESTTEST")
public class TESTTEST extends LinearOpMode{

    MainBaseRight base = new MainBaseRight();
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

        base.yawPIDController.yawReset();




        waitForStart();

        base.findLine(5,this);




        base.navx_device.close();
        base.yawPIDController.enable(false);

    }
}
