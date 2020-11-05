package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class AutoReplayBase extends AutoBase {
    @Override
    protected void preStart() {
        super.preStart();
        String routeName = getModeName();
        if (!routeName.isEmpty()) {
            loadRoute(routeName);
        }
    }

    @Override
    protected void act() {
        super.act();
        runRoute();
    }

    protected String getModeName() {
        Class<?> klass = this.getClass();
        Autonomous annotation =  klass.getAnnotation(Autonomous.class);
        if (annotation != null && annotation.group().equals("playback")){
            return annotation.name();
        }
        return "";
    }
}
