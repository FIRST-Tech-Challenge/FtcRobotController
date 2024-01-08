package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Evolution Reset", group="Evolution")
//@Disabled
public class EvolutionReset extends LinearOpMode {
    WolfNet[] networks;

    @Override
    public void runOpMode() {
        networks = new WolfNet[]{new WolfNet(8, 6, 2, "Weights0", 0.5), new WolfNet(8, 6, 2, "Weights1", 0.5), new WolfNet(8, 6, 2, "Weights2", 0.5), new WolfNet(8, 6, 2, "Weights3", 0.5), new WolfNet(8, 6, 2, "Weights4", 0.5)};
        for(int i = 0; i < 5; i++){
            networks[i].ResetWeights();
            networks[i].SaveWeights();
        }
    }
}