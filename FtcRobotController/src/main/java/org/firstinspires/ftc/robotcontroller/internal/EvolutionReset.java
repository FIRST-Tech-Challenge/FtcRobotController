/* Copyright © 2023 North Paulding High School Robotics Team 16757 */

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Evolution Reset (DO NOT RUN!!!)", group="Evolution Reset")
//@Disabled
public class EvolutionReset extends LinearOpMode {
    ElapsedTime delay = new ElapsedTime();
    boolean delayStart = false;

    WolfNet[] networks;

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a && !delayStart){
                delay.reset();
                delayStart = true;
            }else if(gamepad1.b){
                requestOpModeStop();
            }else if(delayStart){
                if(delay.time() >= 5){
                    networks = new WolfNet[]{new WolfNet(12, 8, 2, "Weights0", 0.5),
                            new WolfNet(12, 8, 2, "Weights1", 0.5),
                            new WolfNet(12, 8, 2, "Weights2", 0.5),
                            new WolfNet(12, 8, 2, "Weights3", 0.5),
                            new WolfNet(12, 8, 2, "Weights4", 0.5)};
                    for(int i = 0; i < 5; i++){
                        networks[i].ResetWeights();
                        networks[i].SaveWeights();
                    }
                    requestOpModeStop();
                }else{
                    telemetry.addData("Deleting in", Math.round(5 - delay.time()));
                    telemetry.addLine();
                    telemetry.addData("To cancel press", "(B)");
                }
            }else{
                telemetry.addData("WARNING!", "This will delete all training data PERMANENTLY! Are you sure you want to continue?");
                telemetry.addLine();
                telemetry.addData("YES", "(A)");
                telemetry.addData("NO", "(B)");
            }

            telemetry.update();
        }
    }
}