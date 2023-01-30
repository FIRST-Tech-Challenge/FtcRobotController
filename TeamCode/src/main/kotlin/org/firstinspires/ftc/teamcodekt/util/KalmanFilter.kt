package org.firstinspires.ftc.teamcodekt.util

/**
 * A Kalman Filter for 1D data.
 * It can be used for multiple dimensioned data, however a separate
 * object must be created for each dimension.

 * @author TLindauer
 */
class KalmanFilter {
    /**
     * Vector A
     */
    private var A = 1.0

    /**
     * Vector B
     */
    private var B = 0.0

    /**
     * Vector C
     */
    private var C = 1.0

    /**
     * Process noise
     */
    private var R: Double

    /**
     * Measurement noise
     */
    private var Q: Double

    /**
     * Covariance variable
     */
    private var cov = Double.NaN

    /**
     * State variable
     */
    private var x = Double.NaN

    /**
     * Instead of specifying a deviceCode, make a custom Kalman Filter.
     * @param R is process noise
     * @param Q is measurement noise
     * @param A is state vector
     * @param B is control vector
     * @param C is measurement vector
     */
    constructor(R: Double, Q: Double, A: Double, B: Double, C: Double) {
        this.R = R
        this.Q = Q
        this.A = A
        this.B = B
        this.C = C
        cov = Double.NaN
        x = Double.NaN // estimated signal without noise
    }

    /**
     * Only specify noise
     * @param R is process noise
     * @param Q is measurement noise
     */
    // R is process noise, Q is measurement noise. No specified state/control/measurement vectors, set to default 1,0,1
    constructor(R: Double, Q: Double) {
        this.R = R
        this.Q = Q
    }

    /**
     * Feed a new value into the Kalman filter and return what the predicted state is.
     * @param measurement the measured value
     * @param u is the controlled input value
     * @return the predicted result.
     * Postcondition: the appropriate filtered value has been returned
     */
    // Filter a measurement: measured value is measurement, controlled input value is u.
    fun filter(measurement: Double, u: Double): Double {
        if (java.lang.Double.isNaN(x)) {
            x = 1 / C * measurement
            cov = 1 / C * Q * (1 / C)
        } else {
            val predX = A * x + B * u
            val predCov = A * cov * A + R

            // Kalman gain
            val K = predCov * C * (1 / (C * predCov * C + Q))

            // Correction
            x = predX + K * (measurement - C * predX)
            cov = predCov - K * C * predCov
        }
        return x
    }

    /**
     * Feed a new value into the Kalman filter and return what the predicted state is.
     * @param measurement the measured value
     * @return the predicted result.
     * Postcondition: the appropriate filtered value has been returned
     */
    // Filter a measurement taken
    fun filter(measurement: Double): Double {
        val u = 0.0
        if (java.lang.Double.isNaN(x)) {
            x = 1 / C * measurement
            cov = 1 / C * Q * (1 / C)
        } else {
            val predX = A * x + B * u
            val predCov = A * cov * A + R

            // Kalman gain
            val K = predCov * C * (1 / (C * predCov * C + Q))

            // Correction
            x = predX + K * (measurement - C * predX)
            cov = predCov - K * C * predCov
        }
        return x
    }

    /**
     * Return the last measurement taken.
     * @return the last measurement
     * Postcondition: returns the last measurement accurately
     */
    // Return the last measurement taken
    fun lastMeasurement(): Double {
        return x
    }

    /**
     * Set the measurement noise
     * @param noise the measurement noise.
     * Postcondition: sets the measurement noise accurately
     */
    // Set measurement noise
    fun setMeasurementNoise(noise: Double) {
        Q = noise
    }

    /**
     * Set the process noise
     * @param noise the process noise.
     * Postcondition: sets the process noise accurately
     */
    fun setProcessNoise(noise: Double) {
        R = noise
    }
}