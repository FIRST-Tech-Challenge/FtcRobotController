package org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The navXPerformanceMonitor class is designed to provide performance
 * data to help tune the navx_ftc library's AHRS_OLD class to retrieve
 * navX-Model device data using tuning parameters appropriate for a
 * particular FTC robotic controller configuration.
 *
 * For an example of using the navXPerformanceMonitor class, see the
 * <a href="https://github.com/kauailabs/navxmxp/blob/master/android/OpModes/navXPerformanceTuningOp.java">
 * navX Performance Tuning Opmode</a>
 */
public class navXPerformanceMonitor implements IDataArrivalSubscriber {

    private ElapsedTime runtime = new ElapsedTime();
    private AHRS navx_device;
    private long last_system_timestamp = 0;
    private long last_sensor_timestamp = 0;
    private long sensor_timestamp_delta = 0;
    private long system_timestamp_delta = 0;
    private byte sensor_update_rate_hz = 40;
    private int missing_sensor_sample_count = 0;
    private int estimated_missing_sensor_sample_count = 0;
    private boolean first_sample_received = false;
    private int hertz_counter = 0;
    private int last_second_hertz = 0;

    final int MS_PER_SEC = 1000;

    /**
     * Constructor for the navXPerformanceMonitor class.
     * @param navx_device  The instance of the navX-Model device to monitor.
     */
    public navXPerformanceMonitor(AHRS navx_device) {
        this.navx_device = navx_device;
        reset();
    }

    /**
     * Resets the peformance monitoring statistics.
     */
    public void reset() {
        last_system_timestamp = 0;
        last_sensor_timestamp = 0;
        sensor_timestamp_delta = 0;
        system_timestamp_delta = 0;
        sensor_update_rate_hz = 40;
        missing_sensor_sample_count = 0;
        first_sample_received = false;
        hertz_counter = 0;
        last_second_hertz = 0;
    }

    /**
     * The delivered rate is the rate at which samples are delivered
     * to the Android-based FTC robotics control application.
     * @return The rate of navX-Model device sample delivery in Hz.
     */
    public int getDeliveredRateHz() {
        return last_second_hertz;
    }

    /**
     * The sensor tate is the rate at which the navX-Model device sensor
     * is currently configured to deliver sample.  This rate may be
     * greater than the delivered rate.
     * @return The rate at which the navX-Model device is currently configured
     * to generate samples, in Hz.
     */
    public int getSensorRateHz() {
        return navx_device.getActualUpdateRate();
    }
    /** The rate at which the Core Device Interface Module (DIM) is currently
     * delivering data samples to the navx_ftc library IO thread.
     * @return The current DIM transfer rate in Hz.
     */
    public int getDimTransferRateHz() {
        return navx_device.getCurrentTransferRate();
    }

    /**
     * The number of samples which were expected to be received from the
     * navX-Model device which never arrived, since the last time the
     * navXPerformanceMonitor's statistics were reset.
     * @return The number of navX-Model device samples not received.
     */
    public int getNumMissedSensorTimestampedSamples() {
        return missing_sensor_sample_count;
    }

    /**
     * The number of samples which were expected to be received from the
     * navX-Model device which are believed to have not arrived based upon
     * the Android OS system timestamp.  Note that this timestamp is not
     * as accurate as the navX-Model device sensor timestamp, and thus the
     * number of missed unstimestamped samples is an estimate, and is not
     * deterministic.
     * @return The estimated number of navX-Model device untimestamped
     * samples not received, since the last time the navXPerformanceMonitor's
     * statistics were reset.
     */
    public int getNumEstimatedMissedUntimestampedSamples() {
        return estimated_missing_sensor_sample_count;
    }

    /**
     * The last sensor timestamp delta indicates the period of time between
     * reception of the last two samples from the navX-Model device, for those
     * data samples which are timestamped with a sensor timestamp.
     * @return The last sensor timestamp delta, in milliseconds.
     */
    public long getLastSensorTimestampDeltaMS() {
        return sensor_timestamp_delta;
    }
    /**
     * The last system timestamp delta indicates the period of time between
     * reception of the last two samples from the navX-Model device, for those
     * data samples which do not provide a corresponding sensor timestamp and
     * are instead timestamped using the Android OS system timestamp.
     * @return The last system timestamp delta, in milliseconds.
     */
    public long getLastSystemTimestampDeltaMS() {
        return system_timestamp_delta;
    }


    private int NORMAL_DIM_TRANSFER_ITTER_MS = 10;
    @Override
    public void untimestampedDataReceived(long curr_system_timestamp, Object kind) {
        byte sensor_update_rate = navx_device.getActualUpdateRate();
        long num_dropped = 0;
         system_timestamp_delta = curr_system_timestamp - last_system_timestamp;
        int expected_sample_time_ms = MS_PER_SEC / (int)sensor_update_rate;

        if ( !navx_device.isConnected() ) {
            reset();
        } else {
            if ( ( curr_system_timestamp % 1000 ) < ( last_system_timestamp % 1000 ) ) {
                /* Second roll over.  Start the Hertz accumulator */
                last_second_hertz = hertz_counter;
                hertz_counter = 1;
            } else {
                hertz_counter++;
            }
            if ( !first_sample_received ) {
                last_sensor_timestamp = curr_system_timestamp;
                first_sample_received = true;
                estimated_missing_sensor_sample_count = 0;
            } else {
                if (system_timestamp_delta > (expected_sample_time_ms + NORMAL_DIM_TRANSFER_ITTER_MS) ) {
                    long estimated_dropped_samples = (system_timestamp_delta / expected_sample_time_ms) - 1;
                    if (estimated_dropped_samples > 0) {
                        estimated_missing_sensor_sample_count += estimated_dropped_samples;
                    }
                }
            }
        }

        last_system_timestamp = curr_system_timestamp;
    }

    final int NAVX_TIMESTAMP_JITTER_MS = 2;

    @Override
    public void timestampedDataReceived(long curr_system_timestamp,
                                        long curr_sensor_timestamp,
                                        Object kind) {
        long num_dropped = 0;
        byte sensor_update_rate = navx_device.getActualUpdateRate();
        sensor_timestamp_delta = curr_sensor_timestamp - last_sensor_timestamp;
        system_timestamp_delta = curr_system_timestamp - last_system_timestamp;
        int expected_sample_time_ms = MS_PER_SEC / (int)sensor_update_rate;

        if ( !navx_device.isConnected() ) {
            reset();
        } else {
            if ( ( curr_system_timestamp % 1000 ) < ( last_system_timestamp % 1000 ) ) {
                /* Second roll over.  Start the Hertz accumulator */
                last_second_hertz = hertz_counter;
                hertz_counter = 1;
            } else {
                hertz_counter++;
            }
            if ( !first_sample_received ) {
                last_sensor_timestamp = curr_sensor_timestamp;
                first_sample_received = true;
                missing_sensor_sample_count = 0;
            } else {
                if (sensor_timestamp_delta > (expected_sample_time_ms + NAVX_TIMESTAMP_JITTER_MS) ) {
                    long dropped_samples = (sensor_timestamp_delta / expected_sample_time_ms) - 1;
                    if (dropped_samples > 0) {
                        missing_sensor_sample_count += dropped_samples;
                    }
                }
            }
        }

        last_sensor_timestamp = curr_sensor_timestamp;
        last_system_timestamp = curr_system_timestamp;
    }

    @Override
    public void yawReset() {
    }
}
