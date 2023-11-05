package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.robot.Robot;
public class CachingSensor<T> implements Robot.Listener {
    public interface UpdateCall<T> {
        T call();
    }

    private UpdateCall<T> call;
    private T value;
    private boolean enabled = true;

    public CachingSensor (UpdateCall<T> call) {
        this.call = call;
    }

    @Override
    public void update () {
        if (enabled) value = call.call();
    }

    public T getValue () {
        return value;
    }

    public void setEnabled (boolean enabled) {
        this.enabled = enabled;
    }
}