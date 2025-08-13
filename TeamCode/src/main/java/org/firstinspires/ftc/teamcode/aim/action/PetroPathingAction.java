package org.firstinspires.ftc.teamcode.aim.action;

import java.util.ArrayList;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Path;

public class PetroPathingAction  implements Action {
    private Follower follower;
    private Path  drivePath;
    private boolean started;
    private boolean holdEnd;
    public PetroPathingAction(Follower follower, Path drivePath, boolean holdEnd) {
        this.follower = follower;
        this.drivePath = drivePath;
        this.holdEnd = holdEnd;
    }

    public boolean run() {
        if (!this.started) {
            this.follower.followPath(this.drivePath,this.holdEnd);
            this.started = true;
        }
        return !this.follower.isBusy();
    }
}
