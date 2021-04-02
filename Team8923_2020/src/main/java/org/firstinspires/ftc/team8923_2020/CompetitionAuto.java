package org.firstinspires.ftc.team8923_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="CompetitionAuto")
public class CompetitionAuto extends MasterAutonomous {
    @Override
    public void runOpMode() throws InterruptedException{
        configureAutonomous();
        initAuto();
        double referenceAngle = imu.getAngularOrientation().firstAngle;
        telemetry.clear();
        telemetry.update();

        waitForStart();
        telemetry.clear();


        while(opModeIsActive()){

            wait(delays * 1000);



            switch (destination){
                case SQUAREA:
                    if(alliance == Alliance.RED){
                        moveAuto(0, 5, 1, .2);
                        imuPivot(referenceAngle, 90, .5, .015, 1);
                        //wobbleGrabberOpen();

                    }
                    break;
                case SQUAREB:
                    if(alliance == Alliance.RED){
                        moveAuto(0, 5, 1, 2.);
                        //runIntake();
                        //turnOffIntake();
                        moveAuto(0, 5, 1, .2);
                        imuPivot(referenceAngle, 90, .5, .015, 1);
                        //wobbleGrabberOpen();
                    }
                    break;
                case SQUAREC:
                    if(alliance == Alliance.RED){
                        moveAuto(0, 5, 1, .2);
                        //runIntake();
                        //turnOffIntake();
                        moveAuto(0, 3, 1, .2 );
                        //wobbleGrabberOpen();
                    }
                    break;
            }




        }


    }
}
