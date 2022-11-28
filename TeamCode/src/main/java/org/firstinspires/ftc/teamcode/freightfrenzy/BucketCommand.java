package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.arcrobotics.ftclib.command.CommandBase;

public class BucketCommand extends CommandBase {
    private final BucketSubsystem bucket;
    private BucketSubsystem.State bucketState;

    public BucketCommand(BucketSubsystem bucket) {
        this.bucket = bucket;
        addRequirements(this.bucket);
    }

    @Override
    public void initialize() {
        bucketState = bucket.getState();

        if (bucketState == BucketSubsystem.State.RELEASE)
            bucket.rest();
        else if (bucketState == BucketSubsystem.State.REST)
            bucket.release();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

