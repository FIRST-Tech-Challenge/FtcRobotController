package org.firstinspires.ftc.teamcode.PurePursuit.Base.Controllers;

import org.firstinspires.ftc.teamcode.PurePursuit.Base.Filters.IIRFilter;

public class PIDFEx implements Controller{

        private IIRFilter filter;
        private double integralWorkingBounds = Double.POSITIVE_INFINITY, integralClippingBounds = Double.POSITIVE_INFINITY;

        private double kP, kI, kD, kF;
        private double alpha;
        private double setPoint;
        private double measuredValue;
        private double minIntegral, maxIntegral;

        private double errorVal_p;
        private double errorVal_p_filtered;
        private double errorVal_v;

        private double totalError;
        private double prevErrorVal_filtered;

        private double errorTolerance_p = 0.05;
        private double errorTolerance_v = Double.POSITIVE_INFINITY;

        private double lastTimeStamp;
        private double period;

        private double deadzone;

        /**
         * The base constructor for the PIDF controller
         */
        public PIDFEx (double kp, double ki, double kd, double kf, double alpha, double deadzone, double integralWorkingBounds, double integralClippingBounds) {
            this(kp, ki, kd, kf, 0, 0, alpha, deadzone, integralWorkingBounds, integralClippingBounds);
        }

        /**
         * This is the full constructor for the PIDF controller. Our PIDF controller
         * includes a feed-forward value which is useful for fighting friction and gravity.
         * Our errorVal represents the return of e(t) and prevErrorVal is the previous error.
         *
         * @param sp The setpoint of the pid control loop.
         * @param pv The measured value of he pid control loop. We want sp = pv, or to the degree
         *           such that sp - pv, or e(t) < tolerance.
         */
        public PIDFEx(double kp, double ki, double kd, double kf, double sp, double pv, double alpha, double deadzone, double integralWorkingBounds, double integralClippingBounds) {
            kP = kp;
            kI = ki;
            kD = kd;
            kF = kf;

            setPoint = sp;
            measuredValue = pv;

            minIntegral = -1.0;
            maxIntegral = 1.0;

            this.alpha = alpha;

            lastTimeStamp = 0;
            period = 0;

            errorVal_p = setPoint - measuredValue;

            this.integralWorkingBounds = integralWorkingBounds;
            this.integralClippingBounds = integralClippingBounds;

            filter = new IIRFilter(alpha, () -> errorVal_p);

            this.setIntegrationBounds(-integralClippingBounds, integralClippingBounds);

            this.deadzone = deadzone;

            reset();
        }

        public void reset() {
            totalError = 0;
            prevErrorVal_filtered = 0;
            lastTimeStamp = 0;
        }

        /**
         * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
         *
         * @param positionTolerance Position error which is tolerable.
         */
        public void setTolerance(double positionTolerance) {
            setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
        }

        /**
         * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
         *
         * @param positionTolerance Position error which is tolerable.
         * @param velocityTolerance Velocity error which is tolerable.
         */
        public void setTolerance(double positionTolerance, double velocityTolerance) {
            errorTolerance_p = positionTolerance;
            errorTolerance_v = velocityTolerance;
        }

        /**
         * Returns the current setpoint of the PIDFController.
         *
         * @return The current setpoint.
         */
        public double getSetPoint() {
            return setPoint;
        }

        /**
         * Sets the setpoint for the PIDFController
         *
         * @param sp The desired setpoint.
         */
        public void setSetPoint(double sp) {
            setPoint = sp;
            errorVal_p = setPoint - measuredValue;
            errorVal_p_filtered = filter.get();

            errorVal_v = (errorVal_p_filtered - prevErrorVal_filtered) / period;
        }

        /**
         * Returns true if the error is within the percentage of the total input range, determined by
         * {@link #setTolerance}.
         *
         * @return Whether the error is within the acceptable bounds.
         */
        public boolean atSetPoint() {
            return Math.abs(errorVal_p) < errorTolerance_p
                && Math.abs(errorVal_v) < errorTolerance_v;
        }

        /**
         * @return the PIDF coefficients
         */
        public double[] getCoefficients() {
            return new double[]{kP, kI, kD, kF};
        }

        /**
         * @return the positional error e(t)
         */
        public double getPositionError() {
            return errorVal_p;
        }

        /**
         * @return the tolerances of the controller
         */
        public double[] getTolerance() {
            return new double[]{errorTolerance_p, errorTolerance_v};
        }

        /**
         * @return the velocity error e'(t)
         */
        public double getVelocityError() {
            return errorVal_v;
        }

        /**
         *
         * Calculates the control value, u(t).
         *
         * @param pose
         * @param velocities
         * @return
         */
        @Override
        public double calculate(double[] pose, double[] velocities) {
            double sp = pose[0];
            double pv = pose[1];
            setSetPoint(sp);

            prevErrorVal_filtered = errorVal_p_filtered;

            double currentTimeStamp = (double) System.nanoTime() / 1E9;
            if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
            period = currentTimeStamp - lastTimeStamp;
            lastTimeStamp = currentTimeStamp;

            if (measuredValue == pv) {
                errorVal_p = setPoint - measuredValue;
            } else {
                errorVal_p = setPoint - pv;
                measuredValue = pv;
            }

            errorVal_p_filtered = filter.get();

            if (Math.abs(period) > 1E-6) {
                errorVal_v = (errorVal_p_filtered - prevErrorVal_filtered) / period;
            } else {
                errorVal_v = 0;
            }

        /*
        if total error is the integral from 0 to t of e(t')dt', and
        e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
            if (errorVal_p > -integralWorkingBounds && errorVal_p < integralWorkingBounds) {
                totalError += period * (setPoint - measuredValue);
                totalError = totalError < minIntegral ? minIntegral : Math.min(maxIntegral, totalError);
            } else if (setPoint - measuredValue < errorTolerance_p) {
                totalError = 0;
            }

            // returns u(t)
            return Math.abs(getPositionError()) > deadzone ? kP * errorVal_p + kI * totalError + kD * errorVal_v + kF * setPoint : 0;
        }

        public void setPIDF(double kp, double ki, double kd, double kf) {
            kP = kp;
            kI = ki;
            kD = kd;
            kF = kf;
        }

        public void setIntegrationBounds(double integralMin, double integralMax) {
            minIntegral = integralMin;
            maxIntegral = integralMax;
        }

        // used to clear kI gains
        public void clearTotalError() {
            totalError = 0;
        }

        public void setP(double kp) {
            kP = kp;
        }

        public void setI(double ki) {
            kI = ki;
        }

        public void setD(double kd) {
            kD = kd;
        }

        public void setF(double kf) {
            kF = kf;
        }

        public void setAlpha(double newAlpha) {
            alpha = newAlpha;
        }

        public void setWorkingBounds(double bound) {
            integralWorkingBounds = bound;
        }

        public double getP() {
            return kP;
        }

        public double getI() {
            return kI;
        }

        public double getD() {
            return kD;
        }

        public double getF() {
            return kF;
        }

        public double getPeriod() {
            return period;
        }

        public double getAlpha() {
            return alpha;
        }

        public double getWorkingBounds() {
            return integralWorkingBounds;
        }
}
