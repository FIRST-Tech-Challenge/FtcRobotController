package org.firstinspires.ftc.teamcode.aim.action;

import java.util.ArrayList;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Path;

public class PetroPathingAction  extends Action {
    private Follower follower;
    private Path  drivePath;
    private boolean holdEnd;

    public PetroPathingAction(String name, Follower follower, Path drivePath, boolean holdEnd) {
        super(name);
        this.follower = follower;
        this.drivePath = drivePath;
        this.holdEnd = holdEnd;
    }

    @Override
    public boolean run() {
        if (!isStarted()) {
            this.follower.followPath(this.drivePath,this.holdEnd);
            markStarted();
        }
        return !this.follower.isBusy();
    }
}
