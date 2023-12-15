package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class BucketBot {
    public bucketbotDrivetrain drivetrain;
    public BucketbotLid bucketbotLid;

    public BucketBot(HardwareMap hwMap) {
        drivetrain = new bucketbotDrivetrain(hwMap);
        bucketbotLid = new BucketbotLid(hwMap);

    }


}
