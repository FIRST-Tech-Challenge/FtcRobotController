package org.firstinspires.ftc.teamcode.auton;

//import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;

//@Config
@Autonomous(name="Playground")
public class playground extends LinearOpMode {
    public static boolean L = false;
    public static boolean R = false;
    public static boolean M = false;
    MecanumHardAuto r = new MecanumHardAuto();
    @Override
    public void runOpMode() throws InterruptedException {
        r.initRobot(this);


        String loc = L ? "Left" : R ? "Right" : "Mid";
        telemetry.addData("Location", loc);
        telemetry.update();
        waitForStart();

        r.moveInches(0.4, 6);
        r.moveInches(0.5, 3.5/2, Hardware.directions.RIGHT);
        switch (loc){
            default:
                r.moveInches(0.7,28.5);
                r.waiter(750);
                r.moveInches(0.7,-11);
//                r.moveInches(0.6,20, Hardware.directions.RIGHT);
//                r.moveInches(0.6, 32);
//                r.superRotate(0.6,90, Hardware.directions.RIGHT);
//                r.moveInches(0.6, -108);
                break;
            case "Left":
                r.moveInches(0.7, 19);
                r.moveInches(0.7,7.5, Hardware.directions.LEFT);
                r.moveInches(0.6,6);
                r.moveInches(0.6,-5);
//                r.moveInches(0.6,64, Hardware.directions.LEFT);
                break;
            case "Right":
                r.moveInches(0.7, 19);
                r.moveInches(0.7,16.5, Hardware.directions.RIGHT);
                r.moveInches(0.6,6);
                r.moveInches(0.6,-5);
//                r.moveInches(0.6,88, Hardware.directions.LEFT);
                break;
        }

    }
}
