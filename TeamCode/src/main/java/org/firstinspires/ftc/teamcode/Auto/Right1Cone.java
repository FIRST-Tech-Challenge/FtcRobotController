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

@Autonomous(name= "Right1Cone")
public class Right1Cone extends LinearOpMode{

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

        if(base.detectorRight.one){
            base.Arezoo(var.DRIVE_SPEED07, 30, 30, 30, 30, 0, this);
            base.navxgyroStrafe(.5, 10, 10, 10, 10, 0, this);
            base.navxgyroTurn(var.TURN_SPEED03, -85, this);
            base.navxgyroHold(var.GYRO_HOLD_SPEED, -90, var.GYRO_HOLD_TIME, this);
            base.goLift(30, .5);
            base.Arezoo(var.DRIVE_SPEED07, -50, -50, -50, -50, -90, this);
            base.navxgyroTurn(var.TURN_SPEED03, -48, this);
            base.navxgyroHold(.05, -43, 1.2, this);
            base.Arezoo(var.DRIVE_SPEED05, -13.2,-13.2,-13.2,-13.2,-43,this);

            sleep(500);

            base.letGoGirl(this);
            base.Arezoo(var.DRIVE_SPEED05, 13.7,13.7,13.7,13.7,-43,this);
            base.navxgyroTurn(var.TURN_SPEED03, -90, this);


//            base.Arezoo(var.DRIVE_SPEED07, 12, 12, 12, 12, -42, this);
//            base.navxgyroTurn(var.TURN_SPEED03, 0, this);
//            base.navxgyroHold(.05, 0, 1.2, this);
//            base.goLift(-20,.5);
//            base.Arezoo(var.DRIVE_SPEED07, -60, -60, -60, -60, -1, this);
//            base.findLine(var.FIND_LINE_TIME, this);
//            base.goLift(-6, .7);
//            base.getItGirl(this);
//            base.goLift(26, .7);
//            base.Arezoo(var.DRIVE_SPEED07, 25,25,25,25,0,this);
//            base.navxgyroTurn(var.TURN_SPEED03, -140, this);
//            base.navxgyroHold(.05, -145, 1.2, this);
//            base.Arezoo(var.DRIVE_SPEED07, -14.1,-14.1,-14.1,-14.1,-145,this);
//            base.letGoGirl(this);
//            base.Arezoo(var.DRIVE_SPEED07, 14,14,14,14,-142,this);
//            base.navxgyroTurn(var.TURN_SPEED03, -180, this);
//            base.Arezoo(var.DRIVE_SPEED10, -31,-31,-31,-31,-180,this);

        } else if(base.detectorRight.two){
            base.Arezoo(var.DRIVE_SPEED07, 30, 30, 30, 30, 0, this);
            base.navxgyroStrafe(.5, 10, 10, 10, 10, 0, this);
            base.navxgyroTurn(var.TURN_SPEED03, -85, this);
            base.navxgyroHold(var.GYRO_HOLD_SPEED, -90, var.GYRO_HOLD_TIME, this);
            base.goLift(30, .5);
            base.Arezoo(var.DRIVE_SPEED07, -50, -50, -50, -50, -90, this);
            base.navxgyroTurn(var.TURN_SPEED03, -48, this);
            base.navxgyroHold(.05, -43, 1.2, this);
            base.Arezoo(var.DRIVE_SPEED05, -13.2,-13.2,-13.2,-13.2,-43,this);

            sleep(500);

            base.letGoGirl(this);
            base.Arezoo(var.DRIVE_SPEED05, 13.7,13.7,13.7,13.7,-43,this);
            base.navxgyroTurn(var.TURN_SPEED03, 0, this);
            base.navxgyroTurn(var.TURN_SPEED03, 0, this);
            base.Arezoo(var.DRIVE_SPEED05, -24,-24,-24,-24,-43,this);

//            base.Arezoo(var.DRIVE_SPEED07, 12, 12, 12, 12, -42, this);
//            base.navxgyroTurn(var.TURN_SPEED03, 0, this);
//            base.navxgyroHold(.05, 0, 1.2, this);
//            base.goLift(-20,.5);
//            base.Arezoo(var.DRIVE_SPEED07, -60, -60, -60, -60, -1, this);
//            base.findLine(var.FIND_LINE_TIME, this);
//            base.goLift(-6, .7);
//            base.getItGirl(this);
//            base.goLift(26, .7);
//            base.Arezoo(var.DRIVE_SPEED07, 25,25,25,25,0,this);
//            base.navxgyroTurn(var.TURN_SPEED03, -140, this);
//            base.navxgyroHold(.05, -145, 1.2, this);
//            base.Arezoo(var.DRIVE_SPEED07, -14.1,-14.1,-14.1,-14.1,-145,this);
//            base.letGoGirl(this);
//            base.Arezoo(var.DRIVE_SPEED07, 10,10,10,10,-142,this);
//            base.navxgyroTurn(var.TURN_SPEED03, -90, this);



        }else{
            base.Arezoo(var.DRIVE_SPEED07, 30, 30, 30, 30, 0, this);
            base.navxgyroStrafe(.5, 10, 10, 10, 10, 0, this);
            base.navxgyroTurn(var.TURN_SPEED03, -85, this);
            base.navxgyroHold(var.GYRO_HOLD_SPEED, -90, var.GYRO_HOLD_TIME, this);
            base.goLift(30, .5);
            base.Arezoo(var.DRIVE_SPEED07, -50, -50, -50, -50, -90, this);
            base.navxgyroTurn(var.TURN_SPEED03, -48, this);
            base.navxgyroHold(.05, -43, 1.2, this);
            base.Arezoo(var.DRIVE_SPEED05, -13.2,-13.2,-13.2,-13.2,-43,this);

            sleep(500);
            base.letGoGirl(this);
            base.Arezoo(var.DRIVE_SPEED05, 13.7,13.7,13.7,13.7,-43,this);
            base.navxgyroTurn(var.TURN_SPEED03, 0, this);
            base.navxgyroTurn(var.TURN_SPEED03, 0, this);
            base.Arezoo(var.DRIVE_SPEED05, -55,-55,-55,-55,-0,this);
//            base.Arezoo(var.DRIVE_SPEED07, 12, 12, 12, 12, -42, this);
//            base.navxgyroTurn(var.TURN_SPEED03, 0, this);
//            base.navxgyroHold(.05, 0, 1.2, this);
//            base.goLift(-20,.5);
//            base.Arezoo(var.DRIVE_SPEED07, -60, -60, -60, -60, -1, this);
//            base.findLine(var.FIND_LINE_TIME, this);
//            base.goLift(-6, .7);
//            base.getItGirl(this);
//            base.goLift(26, .7);
//            base.Arezoo(var.DRIVE_SPEED07, 25,25,25,25,0,this);
//            base.navxgyroTurn(var.TURN_SPEED03, -140, this);
//            base.navxgyroHold(.05, -145, 1.2, this);
//            base.Arezoo(var.DRIVE_SPEED07, -14.1,-14.1,-14.1,-14.1,-145,this);
//            base.letGoGirl(this);
//            base.Arezoo(var.DRIVE_SPEED07, 14,14,14,14,-142,this);
//            base.navxgyroTurn(var.TURN_SPEED03, -180, this);
//            base.Arezoo(var.DRIVE_SPEED10, 20,20,20,20,-180,this);

        }







        base.navx_device.close();
        base.yawPIDController.enable(false);

    }
}
