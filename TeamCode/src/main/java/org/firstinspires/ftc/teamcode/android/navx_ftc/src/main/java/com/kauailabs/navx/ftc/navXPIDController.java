/* ============================================
 NavX-MXP and NavX-Micro source code is placed under the MIT license
 Copyright (c) 2015 Kauai Labs

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */
package org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc;

/**
 * The navXPIDController implements a timestamped PID controller (designed to deal
 * with the jitter which is typically present in a networked control system scenario).
 * <p>
 * The navXPIDController can use any of the various data sources on a navX-Model device
 * as an input (process variable); when instantiating a navXPIDController simply
 * provide an AHRS class instance and specify which navX-Model device variable you
 * wish to use as the input.  Then, configure the navXPIDController's setPoint,
 * outputRange, whether it should operate in continuous mode or not, and the
 * P, I, D and F coefficients which will be used to calculate the output value.
 * <p>
 * An example of using the navXPIDController to rotate a FTC robot to a target angle is
 * provided <a href="http://navx-micro.kauailabs.com/examples/rotate-to-angle/">online</a>.
 * <p>
 * The general PID algorithm used herein is <a href="https://en.wikipedia.org/wiki/PID_controller">discussed in detail on Wikipedia.</a>
 *
 * <p>
 * In addition to the P,I,D terms, a FeedForward term is optionally available
 * which may be useful in cases where velocity is being controlled (e.g., to
 * achieve a continuous rotational velocity using a yaw rate gyro).  The FeedForward
 * concept is discussed further <a href="http://www.expertune.com/articles/UG2007/PIDControlsPLCEnviron.pdf">here.</a>
 * <p>
 * This algorithm implements two features with respect to the integral gain calculated
 * based on the integral (i) coefficient:
 * <p>
 * - Anti-Windup:  Ensures the integral gain doesn't exceed the min/max output range, as discussed
 *   <a href="http://www.expertune.com/articles/UG2007/PIDControlsPLCEnviron.pdf">here.</a>
 * - Time-Correction:  Adjust the integral gain in cases when timestamps indicate that
 *   data samples were lost.
 * <p>
 * This algorithm implements this feature with respect to the derivative gain, as discussed
 *   <a href="http://www.diva-portal.org/smash/get/diva2:570067/FULLTEXT01.pdf">here.</a>
 */

public class navXPIDController implements IDataArrivalSubscriber {

    public enum TimestampType {SENSOR, SYSTEM};

	 /**
     * The PIDResult class encapsulates the data used by the navXPIDController to
	 * communicate current state to a client of the navXPIDController.  The client
	 * creates the instance of the PIDResult, and continually provides it to the
	 * navxPIDController's waitForNewUpdate() and isNewDataAvailable() methods,
	 * depending upon whether the client wishes to block (wait) for new updates,
	 * or periodically poll to determine when new data is available.
     */
	
    static public class PIDResult {
        public double output;
        public long timestamp;
        public boolean on_target;
        public PIDResult() {
            output = 0.0;
            timestamp = 0;
            on_target = false;
        }
		/** 
		* Returns the timestamp of the last data sample processed by the
		* navXPIDController.
		*/
        public long    getTimestamp() { return timestamp; }
		/** 
		* Returns true if the navXPIDController indicated that it is currently
		* "on target" as defined by the configured tolerances and the most
		* recent input data sample.
		*/
        public boolean isOnTarget()   { return on_target; }
		/**
		* Returns the output value calculated by the navXPIDController which
		* corresponds to the most recent input data sample.
		*/
        public double  getOutput()    { return output; }
    }

    private Object sync_event = new Object();

    /**
     * The navXTimestampedDataSources specifies the navX-Model device
     * sensor data source type used by the navXPIDController as it's input data source.
	 * These data sources are timestamped by the navX-Model device and thus are delivered
	 * with sufficient data (a highly-accurate "sensor timestamp") to allow the 
	 * navXPIDController to compensate for any jitter in the transmission from the 
	 * navX-Model device to the navXPIDController.
     */
	 public enum navXTimestampedDataSource {
        YAW,
        PITCH,
        ROLL,
        COMPASS_HEADING,
        FUSED_HEADING,
        ALTITUDE,
        LINEAR_ACCEL_X,
        LINEAR_ACCEL_Y,
        LINEAR_ACCEL_Z
    }

     /**
     * The navXUntimestampedDataSources specifies the navX-Model device
     * sensor data source type used by the navXPIDController as it's input data source.
	 * These data sources are timestamped with the Android "system" timestamp only, and
	 * thus may not have sufficient data to allow the navXPIDController to compensate 
	 * for any jitter in the transmission from the navX-Model device to the navXPIDController.
     */
	 public enum navXUntimestampedDataSource {
        RAW_GYRO_X,
        RAW_GYRO_Y,
        RAW_GYRO_Z,
        RAW_ACCEL_X,
        RAW_ACCEL_Y,
        RAW_ACCEL_Z,
        RAW_MAG_X,
        RAW_MAG_Y,
        RAW_MAG_Z
    }

    private boolean timestamped = true;
    navXTimestampedDataSource timestamped_src;
    navXUntimestampedDataSource untimestamped_src;
    AHRS navx_device;
    long last_system_timestamp = 0;
    long last_sensor_timestamp = 0;

    /* Error statistics */
    private double error_current    = 0.0;
    private double error_previous   = 0.0;
    private double error_total      = 0.0;

    /* Coefficients */
    private double p;
    private double i;
    private double d;
    private double ff;

    /* Input/Output Clamps */
    private double max_input        = 0.0;
    private double min_input        = 0.0;
    private double max_output       = 1.0;
    private double min_output       = -1.0;

    /** 
	* The ToleranceType enumeration defines the type of tolerance to be used by the
	* navXPIDController to determine whether the controller is "on_target". */
    public enum ToleranceType {
        NONE, PERCENT, ABSOLUTE
    }

    private ToleranceType tolerance_type;
    double tolerance_amount;

    /* Behavior */
    private boolean continuous      = false;
    private boolean enabled         = false;

    private double setpoint         = 0.0;
    private double result           = 0.0;

    @Override
    public void untimestampedDataReceived(long curr_system_timestamp, Object kind) {
        if (enabled && (!timestamped) && (kind.getClass() == AHRS.DeviceDataType.class)) {
            double process_value;
            switch (untimestamped_src) {
                case RAW_GYRO_X:
                    process_value = navx_device.getRawGyroX();
                    break;
                case RAW_GYRO_Y:
                    process_value = navx_device.getRawGyroY();
                    break;
                case RAW_GYRO_Z:
                    process_value = navx_device.getRawGyroZ();
                    break;
                case RAW_ACCEL_X:
                    process_value = navx_device.getRawAccelX();
                    break;
                case RAW_ACCEL_Y:
                    process_value = navx_device.getRawAccelY();
                    break;
                case RAW_MAG_X:
                    process_value = navx_device.getRawMagX();
                    break;
                case RAW_MAG_Y:
                    process_value = navx_device.getRawMagY();
                    break;
                case RAW_MAG_Z:
                    process_value = navx_device.getRawMagZ();
                    break;
                default:
                    process_value = 0.0;
                    break;
            }
            int num_missed_samples = 0; /* TODO */
            last_system_timestamp = curr_system_timestamp;
            double output = this.stepController(process_value, num_missed_samples);
            synchronized (sync_event) {
                sync_event.notify();
            }
        }
    }

    @Override
    public void timestampedDataReceived(long curr_system_timestamp,
                                        long curr_sensor_timestamp, Object kind) {
        if (enabled && timestamped && (kind.getClass() == AHRS.DeviceDataType.class)) {
            double process_value;
            switch (timestamped_src) {
                case YAW:
                    process_value = navx_device.getYaw();
                    break;
                case PITCH:
                    process_value = navx_device.getPitch();
                    break;
                case ROLL:
                    process_value = navx_device.getRoll();
                    break;
                case COMPASS_HEADING:
                    process_value = navx_device.getCompassHeading();
                    break;
                case FUSED_HEADING:
                    process_value = navx_device.getFusedHeading();
                    break;
                case LINEAR_ACCEL_X:
                    process_value = navx_device.getWorldLinearAccelX();
                    break;
                case LINEAR_ACCEL_Y:
                    process_value = navx_device.getWorldLinearAccelY();
                    break;
                case LINEAR_ACCEL_Z:
                    process_value = navx_device.getWorldLinearAccelZ();
                    break;
                default:
                    process_value = 0.0;
                    break;
            }
            int num_missed_samples = 0; /* TODO */
            last_system_timestamp = curr_system_timestamp;
            last_sensor_timestamp = curr_sensor_timestamp;
            double output = this.stepController(process_value, num_missed_samples);
            synchronized (sync_event) {
                sync_event.notify();
            }
        }
    }

    /* If the yaw was just reset, this forms a potential discontinuity in the
       yaw sample stream.  It's possible that if isNewUpdateAvailable() is
       invoked after the yaw is reset but before the next post yaw-reset sample
       has been received, that the PID Controller will indicate TRUE.  Thus,
       when the yaw is reset, set the last sensor timestamp to 0, which
       effecitively causes isNewUpdateAvailable() to return FALSE until
       the next (post yaw-reset) sample is received.

       Additionally, the error terms and the output result are also reset,
       and will be recalculated when the next (post yaw-reset) sample arrives.
     */

    @Override
    public void yawReset() {
        if (timestamped && (timestamped_src == navXTimestampedDataSource.YAW)) {
            this.last_sensor_timestamp = 0;
            error_current = 0.0;
            error_previous = 0.0;
            error_total = 0.0;
            result = 0.0;
        }
    }

	/**
	* This navXPIDController constructor is used when the PID Controller is to be 
	* driven by a navX-Model device input data source which is accompanied by a 
	* high-accuracy "sensor" timestamp.
	* <p>
	* The data source specified automatically determines the navXPIDController's
	* input data range.
	**/
    public navXPIDController(AHRS navx_device, navXTimestampedDataSource src) {
        this.navx_device = navx_device;
        this.timestamped = true;
        this.timestamped_src = src;
        switch ( src ) {
            case YAW:
                this.setInputRange( -180.0, 180.0 );
                break;
            case PITCH:
            case ROLL:
                this.setInputRange( -90.0, 90.0 );
                break;
            case COMPASS_HEADING:
            case FUSED_HEADING:
                this.setInputRange( 0.0, 360.0 );
                break;
            case LINEAR_ACCEL_X:
            case LINEAR_ACCEL_Y:
            case LINEAR_ACCEL_Z:
                this.setInputRange (-2.0, 2.0 );
                break;
        }
        navx_device.registerCallback(this);
    }

	/**
	* This navXPIDController constructor is used when the PID Controller is to be 
	* driven by a navX-Model device input data source which is not accompanied by a 
	* high-accuracy "sensor" timestamp.
	* <p>
	* The data source specified automatically determines the navXPIDController's
	* input data range.
	**/
    public navXPIDController(AHRS navx_device, navXUntimestampedDataSource src) {
        this.navx_device = navx_device;
        this.timestamped = false;
        this.untimestamped_src = src;
        navx_device.registerCallback(this);
    }

    public void close() {
        enable(false);
        navx_device.deregisterCallback(this);
    }

	/**
	* isNewUpdateAvailable() should be called by clients of the navXPIDController
	* which need to "poll" to determine whether new navX-Model device data has 
	* been received.  Whether or not new data has been received, this method
	* returns immediately and does not block.
	* <p>
    * @return Returns true if new data has been received since the last time
	* this function was called, otherwise returns false.  If true, the result
	* will updated to reflect the newly-calculated controller values.
	*/
    public boolean isNewUpdateAvailable(PIDResult result) {
        boolean new_data_available;
        if (enabled &&
                ((timestamped && (result.timestamp < this.last_sensor_timestamp)) ||
                (result.timestamp < this.last_system_timestamp))) {
            new_data_available = true;
            result.on_target = this.isOnTarget();
            result.output = this.get();
            if (timestamped) {
                result.timestamp = last_sensor_timestamp;
            } else {
                result.timestamp = last_system_timestamp;
            }
        } else {
            new_data_available = false;
        }
        return new_data_available;
    }

	/**
	* waitForNewUpdate() should be called by clients of the navXPIDController
	* which want to "wait" until new navX-Model device data has been received.  
	* This method will return immediately only if new data has been received
	* since the last time it was called; otherwise, it will block and not return 
	* until new data has been received, or a specified timeout period has passed.  
	* <p>
    * @return Returns true when new data has been received.  If false is returned,
	* this indicates a timeout has occurred while waiting for new data.
	*/
	public boolean waitForNewUpdate(PIDResult result, int timeout_ms) throws InterruptedException {
        boolean ready = isNewUpdateAvailable(result);
        if (!ready && !Thread.currentThread().isInterrupted()) {
            synchronized (sync_event) {
                sync_event.wait(timeout_ms);
            }
            ready = isNewUpdateAvailable(result);
        }
        return ready;
    }

	/**
	* Returns the current amount of error calculated by the navXPIDController
	* based upon the last navX-Model device data sample.  By definition, this
	* error is equal to the set point minus the last device data sample.
	*/
    public double getError() {
        return error_current;
    }

	/**
	* Used to specify the tolerance used to determine whether the navXPIDController
	* is "on_target".  The tolerance threshold is defined by the tolerance_type and
	* the tolerance amount.  For example, if the YAW data source is used and thus the
	* values used are in units of degrees, ToleranceType.ABSOLUTE and tolerance_amount
	* of 2.0 would result in the navXPIDController deciding it was "on_target" whenever
	* the error was within a range of +/- 2 degrees.
	*/
    public synchronized void setTolerance(ToleranceType tolerance_type, double tolerance_amount) {
        this.tolerance_amount = tolerance_amount;
        this.tolerance_type = tolerance_type;
    }

	/**
	* Indicates whether the navXPIDController has determined that it is "on_target" based upon
	* whether the current error is within the range defined by the tolerance.
	*/
    public boolean isOnTarget() {
        boolean on_target = false;
        switch (tolerance_type) {
            case NONE:
                on_target = (getError() == 0);
                break;
            case PERCENT:
                on_target = (Math.abs(getError()) <
                        (tolerance_amount / 100 * (max_input - min_input)));
                break;
            case ABSOLUTE:
                on_target = (Math.abs(getError()) < tolerance_amount);
                break;
        }
        return on_target;
    }

	/**
	* stepController() is the PIDController worker function, which is used internally
	* by the navXPIDController whenever a new navX-Model device sensor data value
	* is received.
	*/
    public double stepController(double process_variable, int num_missed_samples) {
        double local_result;
        double i_adj;
        double d_adj;

        synchronized (this) {

            error_current = setpoint - process_variable;

            /* If a continuous controller, if > 1/2 way from the target */
            /* modify the error to ensure the output doesn't change     */
            /* direction, but processed onward until error is zero.     */
            if (continuous) {
                double range = max_input - min_input ;
                if (Math.abs(error_current) > (range / 2))  {
                    if (error_current > 0)
                        error_current -= range;
                    else
                        error_current += range;
                }
            }

            /* If samples were missed, reduce the d gain by dividing */
            /* by the total number of samples since the last update. */
            /* For more discussion on this topic, see:               */
            /* http://www.diva-portal.org/smash/get/diva2:570067/FULLTEXT01.pdf */
            if ( num_missed_samples > 0 ) {
                i_adj = i / (1 + num_missed_samples);
            } else {
                i_adj = i;
            }
            /* Process integral term.  Perform "anti-windup" processing */
            /* by preventing the integral term from accumulating when   */
            /* the output reaches it's minimum or maximum limits.       */

            if (i != 0) {
                double estimated_i = (error_total + error_current) * i_adj;
                if (estimated_i < max_output) {
                    if (estimated_i > min_output) {
                        error_total += error_current;
                    } else {
                        error_total = min_output / i_adj;
                    }
                } else {
                    error_total = max_output / i_adj;
                }
            }

            /* If samples were missed, reduce the d gain by dividing */
            /* by the total number of samples since the last update. */
            if ( num_missed_samples > 0 ) {
                d_adj = d / (1 + num_missed_samples);
            } else {
                d_adj = d;
            }

            /* Calculate result w/P, I, D & F terms */
            result = p     * error_current +
                    i_adj  * error_total +
                    d_adj  * (error_current - error_previous) +
                    ff     * setpoint;
            error_previous = error_current;

            /* Clamp result to output range */
            if (result > max_output) {
                result = max_output;
            } else if (result < min_output) {
                result = min_output;
            }

            local_result = result;
        }
        return local_result;
    }

	/**
	* setPID() is used to set the Proportional, Integral and Differential coefficients
	* used by the navXPIDController.
	*/
    public synchronized void setPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
        stepController(error_previous, 0); /* Update cached values */
    }

	/**
	* setPID() is used to set the Proportional, Integral, Differential and FeedForward
	* coefficients used by the navXPIDController.
	*/
    public synchronized void setPID(double p, double i, double d, double ff) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.ff = ff;
        stepController(error_previous, 0); /* Update cached values */
    }

	/**
	* setContinuous() is used to enable/disable the continuous mode of operation.
	*
	* When continuous mode is disabled, the min/max input range values are used as
	* two separate points at the ends of the range of possible input values.  This
	* mode of operation is typically used for reaching a position within a linear
	* range.
	*
	* When continuous mode is enabled, the min/max input range are considered to
	* represent the sampe point.  This mode of operation is typically used for reaching
	* a position within a circular range, and allows the navXPIDController to determine
	* the shortest possible route to the setpoint.  For example, when using YAW as
	* the input data source, and using the PID controller to rotatie to a given angle,
	* if the setpoint is 150 degrees and the current input value is -150 degrees, the 
	* controller will calculate an output such that it travels across the boundary
	* between -180 and 180 degrees (for a total traveled distance of 60 degrees), rather
	* than traveling 300 degrees as it would if continuous mode were disabled. 
	*/
    public synchronized void setContinuous(boolean continuous) {
        this.continuous = continuous;
        stepController(error_previous, 0); /* Update cached values */
    }

	/**
	* Returns the current output value calculated by the navXPIDController based upon the
	* last navX-Model device data sample received.
	*/
    public synchronized double get() {
        return result;
    }

	/**
	* Defines the range of output values calculated by the navXPIDController.
	*
	* For example, when the navXPIDController is used to calculate an output value to be
	* sent to a motor controller whose valid range is -1 to 1, the output range should be
	* sent to -1, 1.
	*
	* Note that the units of the output range are not necessarily the same as the units
	* of the input range.
	*/
    public synchronized void setOutputRange(double min_output, double max_output) {
        if (min_output <= max_output) {
            this.min_output = min_output;
            this.max_output = max_output;
        }
        stepController(error_previous, 0); /* Update cached values */
    }

	/**
	* Defines the range of possible input values received from the currently-selected
	* navX-Model device data source.  For example, if YAW is the data source, the 
	* input range would be -180.0, 180.0.
	*
	* Note that the navXPIDController constructor automatically sets the input range
	* based upon the data source specified to the constructor.
	*/
    public synchronized void setInputRange(double min_input, double max_input) {
        if (min_input <= max_input) {
            this.min_input = min_input;
            this.max_input = max_input;
            setSetpoint(setpoint);
        }
    }

	/**
	* Defines the "target" value the navXPIDController attempts to reach.  This value 
	* is in the same units as the input, and should be within the input range.  For
	* example, if YAW is the data source, the setput should be between -180.0 and 180.0.
	*/
    public synchronized void setSetpoint(double setpoint) {
        if (max_input > min_input) {
            if (setpoint > max_input) {
                this.setpoint = max_input;
            } else if (setpoint < min_input) {
                this.setpoint = min_input;
            } else {
                this.setpoint = setpoint;
            }
        } else {
            this.setpoint = setpoint;
        }
        stepController(error_previous, 0); /* Update cached values */
    }

	/**
	* Returns the currently configured setpoint.
	*/
    public synchronized double getSetpoint() {
        return setpoint;
    }

	/**
	* Enables/disables the PID controller.  By default, the navXPIDController is
	* disabled, thus this method must be invoked before attempting to
	* use the navXPIDController's output values.
	*/
    public synchronized void enable(boolean enabled) {
        this.enabled = enabled;
        if ( !enabled ) {
            reset();
        }
    }

	/**
	* Returns true if the navXPIDController is currently enabled, otherwise
	* return false.
	*/
    public synchronized boolean isEnabled() {
        return this.enabled;
    }

	/**
	* Resets the PIDController's current error value as well as the integrated
	* error.  Also disables the controller.  The enable() method must be used
	* to re-enable the controller.
	*/
    public synchronized void reset() {
        this.enabled = false;
        error_current = 0.0;
        error_previous = 0.0;
        error_total = 0.0;
        result = 0.0;
    }
}
