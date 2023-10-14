package org.firstinspires.ftc.teamcode.lib.util;

import static org.firstinspires.ftc.teamcode.lib.util.TimeUtil.getMillisecondsPerSecond;
import static org.firstinspires.ftc.teamcode.lib.util.TimeUtil.getNanosecondsPerMillisecond;
import static org.firstinspires.ftc.teamcode.lib.util.TimeUtil.getSecondsPerMinute;

public enum TimeUnits implements Units<TimeUnits> {
    MINUTES,
    SECONDS,
    MILLISECONDS,
    NANOSECONDS;

    @Override
    public double in(final TimeUnits convertTo, final double value) {
        return (ordinal() == convertTo.ordinal() ? 1 : (isIn(MINUTES) ? (convertTo.isIn(SECONDS) ? getSecondsPerMinute() :
                convertTo.isIn(MILLISECONDS) ? getSecondsPerMinute() * getMillisecondsPerSecond() :
                        getSecondsPerMinute() * getMillisecondsPerSecond() * getNanosecondsPerMillisecond()) : isIn(SECONDS) ?
                (convertTo.isIn(MINUTES) ? 1 / getSecondsPerMinute() :
                        convertTo.isIn(MILLISECONDS) ? getMillisecondsPerSecond() : getMillisecondsPerSecond() * getNanosecondsPerMillisecond()) :
                isIn(MILLISECONDS) ? (convertTo.isIn(MINUTES) ? 1 / getMillisecondsPerSecond() / getSecondsPerMinute() :
                        convertTo.isIn(SECONDS) ? 1 / getMillisecondsPerSecond() : getNanosecondsPerMillisecond()) :
                        convertTo.isIn(MINUTES) ? 1 / getNanosecondsPerMillisecond() / getMillisecondsPerSecond() / getSecondsPerMinute() :
                                convertTo.isIn(SECONDS) ? 1 / getNanosecondsPerMillisecond() / getMillisecondsPerSecond() :
                                        1 / getNanosecondsPerMillisecond())) * value;
    }

    private boolean isIn(TimeUnits units) {
        return ordinal() == units.ordinal();
    }
}