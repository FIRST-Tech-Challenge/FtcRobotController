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
                        collectRings(0);
                        moveAuto(0, 5, 1, .2);
                        imuPivot(referenceAngle, 90, .5, .015, 1);
                        wobbleUp();


                    break;
                case SQUAREB:
                        collectRings(1);
                        moveAuto(0, 5, 1, 2.);
                        moveAuto(0, 5, 1, .2);
                        imuPivot(referenceAngle, 90, .5, .015, 1);
                        wobbleDown();
                        runShooter();
                        turnOffShooter();


                    break;
                case SQUAREC:
                        collectRings(4);
                        moveAuto(0, 3, 1, .2 );
                        wobbleUp();
                        runShooter();
                        turnOffShooter();

                    break;
            }




        }


    }
}
