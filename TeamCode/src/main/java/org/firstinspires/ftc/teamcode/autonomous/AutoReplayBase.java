package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class AutoReplayBase extends AutoBase {
    @Override
    protected void preStart() {
        super.preStart();
        String routeName = getModeName();
        if (!routeName.isEmpty()) {
            loadRoute(routeName);
            if (this.selectedRoute != null){
                this.setOpModeSide(this.selectedRoute.getName());
            }
        }
    }

    @Override
    protected void act() {
        super.act();
        if (opModeIsActive()) {
            runRoute();
        }
    }

    protected String getModeName() {
        Class<?> klass = this.getClass();
        Autonomous annotation =  klass.getAnnotation(Autonomous.class);
        if (annotation != null && annotation.group().equals("playback")){
            String fullName = annotation.name();
            int numOfDashes = fullName.length() - fullName.replace("-", "").length();
            if (numOfDashes > 1){
                int last = fullName.lastIndexOf("-");
                fullName = fullName.substring(0, last);
            }
            return fullName;
        }
        return "";
    }
}
