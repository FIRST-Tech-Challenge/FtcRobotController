//
package org.firstinspires.ftc.teamcode;
//


import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.procedures.Tests;

public class Tuning {
    @Tuner
    public static Procedure MecanumTuner() {
        return new MecanumTuner();
    }

//    @Tuner
//    public static Procedure tests() {
//        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), null, null);
//    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }
     //can do either first doesn't matter.// IS A FOUR BAR ODO THING JUDAH SAID.




    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }
    // Foresight must be done after Mecanum and Pinpoint is complete.
    // MOVE FIVE INCHES NO MATTER WHAT

    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
    }

}

