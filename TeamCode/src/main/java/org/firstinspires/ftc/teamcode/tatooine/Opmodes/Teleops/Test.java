package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Lift;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class Test extends LinearOpMode {

    public double dis(double RGB1[],double RGB2[]){
        return Math.sqrt(Math.pow((RGB2[0]-RGB1[0]),2) + Math.pow((RGB2[1]-RGB1[1]),2)+Math.pow(((RGB2[2]-RGB1[2])),2));
    }
    double trueRed[]= {255,0,0};
    double trueBue[]= {0,0,255};
    double trueYellow[]= {0,255,0};
    @Override
    public void runOpMode() throws InterruptedException {


        RevColorSensorV3 r = hardwareMap.get(RevColorSensorV3.class,"r");
        r.setGain(51);
        double dalta = 40;
        int col;
        double myCol[] = new double[3];
        waitForStart();

        while (opModeIsActive()) {

            myCol[0] = r.getNormalizedColors().red*255;
            myCol[1] = r.getNormalizedColors().green*255;
            myCol[2] = r.getNormalizedColors().blue*255;


            if (myCol[0] > (myCol[1] + dalta) && myCol[0] > (myCol[2] + dalta))
            {
                col = 0;
            }
            else if(myCol[2] > (myCol[0] + dalta) && myCol[2] > (myCol[1] + dalta))
            {
                col = 2;
            }
            else
            {
                col = 1;
            }




            if (col == 0){
                telemetry.addData("red",true);
            }
            else if (col == 1){
                telemetry.addData("yel",true);
            }
            else {
                telemetry.addData("blue", true);
            }



            telemetry.addData("norm r",myCol[0] );
            telemetry.addData("norm g",myCol[1] );
            telemetry.addData("norm b",myCol[2] );
            telemetry.addData("r", r.red());
            telemetry.addData("g", r.green());
            telemetry.addData("b", r.blue());
            telemetry.addData("dis", r.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}

