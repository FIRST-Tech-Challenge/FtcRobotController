package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Base.MainBaseLeft;
import org.firstinspires.ftc.teamcode.Base.VariablesBase;



//Blue Autonomous: Delivers Duck and Parks in WH/
//Starting Position: Back facing Carousel (10 degrees from wall)

@Autonomous(name= "Left1Cone")
public class Left1Cone extends LinearOpMode{

    MainBaseLeft base = new MainBaseLeft();
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

        if(base.detectorLeft.one == true){

            base.Arezoo(var.DRIVE_SPEED07, 30,30,30,30,0,this);
            base.navxgyroStrafe(var.DRIVE_SPEED06,-10,-10,-10,-10,0,this);
            base.navxgyroTurn(var.TURN_SPEED03,90, this);

            base.navxgyroHold(var.GYRO_HOLD_SPEED,91,var.GYRO_HOLD_TIME,this);
            base.Arezoo(var.DRIVE_SPEED07, -25,-25,-25,-25,91,this);
            base.goLift(30, .6);
            base.Arezoo(var.DRIVE_SPEED07, -25,-25,-25,-25,90,this);
            base.navxgyroTurn(var.TURN_SPEED03,41, this);
            base.navxgyroHold(.05,41,1.2,this);
            base.Arezoo(var.DRIVE_SPEED07, -12.4,-12.4,-12.4,-12.4,41,this);
            base.letGoGirl(this);

            base.Arezoo(var.DRIVE_SPEED07, 12.6,12.6,12.6,12.6,41,this);
            base.navxgyroTurn(var.TURN_SPEED03,0, this);
            base.navxgyroHold(.05,0,1.2,this);
            base.Arezoo(var.DRIVE_SPEED07, -60,-60,-60,-60,0,this);

        }else if(base.detectorLeft.two == true){
            base.Arezoo(var.DRIVE_SPEED07, 30,30,30,30,0,this);
            base.navxgyroStrafe(var.DRIVE_SPEED06,-10,-10,-10,-10,0,this);
            base.navxgyroTurn(var.TURN_SPEED03,90, this);

            base.navxgyroHold(var.GYRO_HOLD_SPEED,91,var.GYRO_HOLD_TIME,this);
            base.Arezoo(var.DRIVE_SPEED07, -25,-25,-25,-25,91,this);
            base.goLift(30, .6);
            base.Arezoo(var.DRIVE_SPEED07, -25,-25,-25,-25,90,this);
            base.navxgyroTurn(var.TURN_SPEED03,41, this);
            base.navxgyroHold(.05,41,1.2,this);
            base.Arezoo(var.DRIVE_SPEED07, -12.4,-12.4,-12.4,-12.4,41,this);
            base.letGoGirl(this);


            base.Arezoo(var.DRIVE_SPEED07, 12.6,12.6,12.6,12.6,41,this);
            base.navxgyroTurn(var.TURN_SPEED03,0, this);
            base.navxgyroHold(.05,0,1.2,this);
            base.Arezoo(var.DRIVE_SPEED07, -27,-27,-27,-27,0,this);
        }else{
            base.Arezoo(var.DRIVE_SPEED07, 30,30,30,30,0,this);
            base.navxgyroStrafe(var.DRIVE_SPEED06,-10,-10,-10,-10,0,this);
            base.navxgyroTurn(var.TURN_SPEED03,90, this);

            base.navxgyroHold(var.GYRO_HOLD_SPEED,91,var.GYRO_HOLD_TIME,this);
            base.Arezoo(var.DRIVE_SPEED07, -25,-25,-25,-25,91,this);
            base.goLift(30, .6);
            base.Arezoo(var.DRIVE_SPEED07, -25,-25,-25,-25,90,this);
            base.navxgyroTurn(var.TURN_SPEED03,41, this);
            base.navxgyroHold(.05,41,1.2,this);
            base.Arezoo(var.DRIVE_SPEED07, -12.4,-12.4,-12.4,-12.4,41,this);
            base.letGoGirl(this);
            base.Arezoo(var.DRIVE_SPEED07, 13,13,13,13,-41,this);
            base.navxgyroTurn(var.TURN_SPEED03,90, this);
            base.Arezoo(var.DRIVE_SPEED07, -13,-13,-13,-13,90,this);




        }




//
//        //Cone 1
//        base.Arezoo(var.DRIVE_SPEED07, 30,30,30,30,0,this);
//        base.navxgyroStrafe(var.DRIVE_SPEED06,-10,-10,-10,-10,0,this);
//        base.navxgyroTurn(var.TURN_SPEED03,90, this);
////        telemetry.addData("YAW:", base.navx_device.getYaw());
////        telemetry.update();
////        sleep(5000);
//        base.navxgyroHold(var.GYRO_HOLD_SPEED,91,var.GYRO_HOLD_TIME,this);
//        base.Arezoo(var.DRIVE_SPEED07, -25,-25,-25,-25,91,this);
//        base.goLift(30, .6);
//        base.Arezoo(var.DRIVE_SPEED07, -25,-25,-25,-25,90,this);
//        base.navxgyroTurn(var.TURN_SPEED03,44, this);
//        base.navxgyroHold(.05,41,1.2,this);
//        base.Arezoo(var.DRIVE_SPEED07, -12.6,-12.6,-12.6,-12.6,41,this);
//        base.letGoGirl(this);
//
//        //Cone 2
//        base.Arezoo(var.DRIVE_SPEED07, 16.8,16.8,16.8,16.8,41,this);
//        base.navxgyroTurn(var.TURN_SPEED03,-2, this);
//        base.navxgyroHold(var.GYRO_HOLD_SPEED,-4,var.GYRO_HOLD_TIME,this);
//        base.goLift(-19, .6);
//        base.Arezoo(var.DRIVE_SPEED04, -10,-10,-10,-10,-4,this);
//        base.Arezoo(var.DRIVE_SPEED07, -53,-54,-54,-54,-4,this);
//        base.findLine(var.FIND_LINE_TIME, this);
//        //Grab Cone 2
//        base.goLift(-6, .7);
//        base.getItGirl(this);
//        base.goLift(26, .8);
//        base.Arezoo(var.DRIVE_SPEED07, 28,28,28,28,-1,this);
//        base.navxgyroTurn(.5,124, this);
//        base.navxgyroHold(.05,141,1.2,this);
//        base.Arezoo(var.DRIVE_SPEED07, -10,-10,-10,-10,141,this);
//        base.letGoGirl(this);

        //Parking Zone 2
//        base.Arezoo(var.DRIVE_SPEED10, 10,10,10,10,138,this);
//        base.MRgyroTurn(var.DRIVE_SPEED10, 90,this);
//        base.Arezoo(var.DRIVE_SPEED10, 10, 10,10,10,90,this);

        //Parking Zone 3
//        base.MRgyroDrive(var.DRIVE_SPEED10, 12,12, 12,12,0, this);
//        base.MRgyroTurn(var.DRIVE_SPEED10, 180,this);
//        base.MRgyroDrive(var.DRIVE_SPEED10, -17,-17,-17,-17,180, this);

        //Parking Zone 1
//        base.MRgyroDrive(var.DRIVE_SPEED10, 15,15, 15,15,0, this);
//        base.MRgyroTurn(var.DRIVE_SPEED10, 180,this);
//        base.MRgyroDrive(var.DRIVE_SPEED10, 23,23,23,23,180, this);








        base.navx_device.close();
        base.yawPIDController.enable(false);

    }
}
