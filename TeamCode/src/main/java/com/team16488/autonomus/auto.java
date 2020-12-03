package com.team16488.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team16488.common.RobotMap;
import com.team16488.compoonents.MecanumDrive;

public class auto extends LinearOpMode {
    // variables
    RobotMap                robot   = new RobotMap();
    private final ElapsedTime runtime = new ElapsedTime();
    // load components
    private final MecanumDrive    drive   = new MecanumDrive(this.robot);

    // calling numerical constants
    // public double ex;

    @Override
    public void runOpMode(){
        // initialize hardware
        robot.mapHardware(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // initialize drive
        drive.encoderDriveInit();

        // custom auto code here
        // Note you can add stuff to components and then just call that stuff here if you want
        //ex:
            //drive.operatorMecanumDrive(0,0,1);
            //drive.stopDrive();
            // that would drive then stop with the right if statements between those two methods

        // calling numerical constants
        // ex = NUMERICAL_CONSTANTS.TICKS_PER_INCH;
    }
}
