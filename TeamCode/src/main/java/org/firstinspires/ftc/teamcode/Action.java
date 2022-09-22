package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public interface Action {
    void init();
    default boolean loop(TelemetryPacket p) {
        return false;
    }

    default void draw(Canvas c) {

    }

    default void runBlocking() {
        if (Thread.currentThread().isInterrupted()) return;

        init();

        boolean b = true;
        while (!Thread.currentThread().isInterrupted() && b) {
            TelemetryPacket p = new TelemetryPacket();

            draw(p.fieldOverlay());
            b = loop(p);

            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    default double clock() {
        return 1e-9 * System.nanoTime();
    }

    // not persistent!
    interface IBuilder<T extends IBuilder<T>> {
        T add(Action a);
    }

    // parent-less, generic-less SeqBuilder
    class Builder implements IBuilder<Builder> {
        private final List<Action> as = new ArrayList<>();

        @Override
        public Builder add(Action a) {
            as.add(a);
            return this;
        }

        public SeqBuilder<Builder> seq() {
            return new SeqBuilder<>(this);
        }

        public ParBuilder<Builder> par() {
            return new ParBuilder<>(this);
        }

        public Builder sleep(double duration) {
            return add(new SleepAction(duration));
        }

        public Builder after(double duration, Action a) {
            return seq().sleep(duration).add(a).end();
        }

        public Action build() {
            return new SequentialAction(as);
        }
    }

    class SeqBuilder<T extends IBuilder<T>> implements IBuilder<SeqBuilder<T>> {
        private final T parent;
        private final List<Action> as = new ArrayList<>();

        private SeqBuilder(T parent) {
            this.parent = parent;
        }

        @Override
        public SeqBuilder<T> add(Action a) {
            as.add(a);
            return this;
        }

        public T end() {
            parent.add(new SequentialAction(as));
            return parent;
        }

        public SeqBuilder<SeqBuilder<T>> seq() {
            return new SeqBuilder<>(this);
        }

        public ParBuilder<SeqBuilder<T>> par() {
            return new ParBuilder<>(this);
        }

        public SeqBuilder<T> sleep(double duration) {
            return add(new SleepAction(duration));
        }

        public SeqBuilder<T> after(double duration, Action a) {
            return seq().sleep(duration).add(a).end();
        }
    }

    final class ParBuilder<T extends IBuilder<T>> implements IBuilder<ParBuilder<T>> {
        private final T parent;
        private final List<Action> as = new ArrayList<>();

        private ParBuilder(T parent) {
            this.parent = parent;
        }

        @Override
        public ParBuilder<T> add(Action a) {
            as.add(a);
            return this;
        }

        public T end() {
            parent.add(new ParallelAction(as));
            return parent;
        }

        public SeqBuilder<ParBuilder<T>> seq() {
            return new SeqBuilder<>(this);
        }

        public ParBuilder<ParBuilder<T>> par() {
            return new ParBuilder<>(this);
        }

        public ParBuilder<T> sleep(double duration) {
            return add(new SleepAction(duration));
        }

        public ParBuilder<T> after(double duration, Action a) {
            return seq().sleep(duration).add(a).end();
        }
    }
}

final class SleepAction implements Action {
    public final double duration;
    private double endTs;

    public SleepAction(double duration) {
        this.duration = duration;
    }

    @Override
    public void init() {
        endTs = clock() + duration;
    }

    @Override
    public boolean loop(TelemetryPacket p) {
        return clock() <= endTs;
    }
}

final class ParallelAction implements Action {
    public final List<Action> actions;
    private List<Action> remaining;

    public ParallelAction(List<Action> actions) {
        this.actions = Collections.unmodifiableList(actions);
    }

    public ParallelAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public void init() {
        remaining = new ArrayList<>(actions);
        for (Action a : remaining) {
            a.init();
        }
    }

    @Override
    public boolean loop(TelemetryPacket p) {
        List<Action> newRem = new ArrayList<>();
        for (Action a : remaining) {
            if (a.loop(p)) {
                newRem.add(a);
            }
        }

        remaining = newRem;

        return remaining.size() > 0;
    }

    @Override
    public void draw(Canvas c) {
        for (Action a : actions) {
            draw(c);
        }
    }
}

final class SequentialAction implements Action {
    public final List<Action> actions;
    private int index;
    private boolean needsInit;

    public SequentialAction(List<Action> actions) {
        this.actions = Collections.unmodifiableList(actions);
    }

    public SequentialAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public void init() {
        if (actions.isEmpty()) {
            return;
        }

        actions.get(0).init();
    }

    @Override
    public boolean loop(TelemetryPacket p) {
        if (index >= actions.size()) {
            return false;
        }

        Action a = actions.get(index);

        if (needsInit) {
            a.init();
            needsInit = false;
        }

        if (!a.loop(p)) {
            index++;
            needsInit = true;
        }

        return index < actions.size();
    }

    @Override
    public void draw(Canvas c) {
        for (Action a : actions) {
            draw(c);
        }
    }
}
