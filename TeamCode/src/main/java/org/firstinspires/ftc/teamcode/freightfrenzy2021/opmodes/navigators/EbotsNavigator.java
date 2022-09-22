package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.navigators;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.ebotsutil.Pose;

public interface EbotsNavigator {

    @Nullable
    public Pose getPoseEstimate();


}
