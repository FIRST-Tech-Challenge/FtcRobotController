package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.VariablesBase;


//Blue Autonomous: Delivers Duck and Parks in WH/
//Starting Position: Back facing Carousel (10 degrees from wall)

@Autonomous(name= "Left2Cone")
public class Left2Cone extends LinearOpMode{

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

        base.yawPIDController.yawReset();




        waitForStart();

        //Cone 1
        base.Arezoo(var.DRIVE_SPEED07, 30,30,30,30,0,this);
        base.navxgyroStrafe(var.DRIVE_SPEED05,-10,-10,-10,-10,0,this);
        base.navxgyroTurn(var.TURN_SPEED03,90, this);
//        telemetry.addData("YAW:", base.navx_device.getYaw());
//        telemetry.update();
//        sleep(5000);
        base.navxgyroHold(var.GYRO_HOLD_SPEED,91,var.GYRO_HOLD_TIME,this);
        base.Arezoo(var.DRIVE_SPEED07, -25,-25,-25,-25,91,this);
        base.goLift(30, .6);
        base.Arezoo(var.DRIVE_SPEED07, -25,-25,-25,-25,90,this);
        base.navxgyroTurn(var.TURN_SPEED03,44, this);
        base.navxgyroHold(.05,41,1.2,this);
        base.Arezoo(var.DRIVE_SPEED07, -11.9,-11.9,-11.9,-11.9,41,this);
        base.letGoGirl(this);

        //Cone 2
        base.Arezoo(var.DRIVE_SPEED07, 16,16,16,16,41,this);
        base.navxgyroTurn(var.TURN_SPEED03,0, this);
        base.navxgyroHold(var.GYRO_HOLD_SPEED,-1,var.GYRO_HOLD_TIME,this);
        base.goLift(-19, .6);
        base.Arezoo(0.3, -10,-10,-10,-10,-1,this);

        base.Arezoo(var.DRIVE_SPEED07, -51,-51,-51,-51,-1,this);
        base.findLine(var.FIND_LINE_TIME, this);
        base.goLift(-6, .7);
        base.getItGirl(this);
        base.goLift(26, .8);
        base.Arezoo(var.DRIVE_SPEED07, 28,28,28,28,-1,this);
        base.navxgyroTurn(.5,121, this);
        base.navxgyroHold(.05,139,1.2,this);
        base.Arezoo(var.DRIVE_SPEED07, -9.5,-9.5,-9.5,-9.5,139,this);
        base.letGoGirl(this);

        //Parking
        base.Arezoo(var.DRIVE_SPEED10, 12,12,12,12,139,this);
        base.navxgyroTurn(var.DRIVE_SPEED07,90, this);
        base.Arezoo(var.DRIVE_SPEED10, 10, 10,10,10,90,this);








        base.navx_device.close();
        base.yawPIDController.enable(false);

    }
}
