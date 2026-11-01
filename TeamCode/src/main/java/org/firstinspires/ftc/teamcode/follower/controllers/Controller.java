/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.controllers;

import java.util.function.Supplier;

@FunctionalInterface
public interface Controller {
    double calculate(double target, double error);

    Controller zero = (t, e) -> 0;

    default double calculate(double target, double error, double derivative) {
        return calculate(target, error);
    }

    default void reset() {}

    /**
     * Returns a PID controller with the given gains.
     */
    static PIDController pid(double kP, double kI, double kD) {
        return new PIDController(kP, kI, kD);
    }

    /**
     * Returns a piecewise controller with a baseline controller
     */
    static PiecewiseController piecewise(Controller baseline) {
        return new PiecewiseController(baseline);
    }

    /**
     * Returns a Controller that combines multiple controllers into one by summing their outputs.
     */
    static Controller sum(Controller... controllers) {
        return new Controller() {
            @Override
            public double calculate(double target, double error) {
                double output = 0;

                for (Controller controller : controllers) {
                    output += controller.calculate(target, error);
                }

                return output;
            }

            @Override
            public double calculate(double target, double error, double derivative) {
                double output = 0;

                for (Controller controller : controllers) {
                    output += controller.calculate(target, error, derivative);
                }

                return output;
            }

            @Override
            public void reset() {
                for (Controller controller : controllers) {
                    controller.reset();
                }
            }
        };
    }

    /**
     * Returns a Controller that provides a constant output in the direction of the error.
     */
    static Controller staticFeedforward(Supplier<Double> kS) {
        return (target, error) -> kS.get() * Math.signum(error);
    }

    /**
     * Returns a Controller that provides a constant output in the direction of the error.
     */
    static Controller staticFeedforward(double kS) {
        return (target, error) -> kS * Math.signum(error);
    }

    /**
     * Returns a Controller that provides a constant output in the direction of the target.
     */
    static Controller staticTargetFeedforward(Supplier<Double> kS) {
        return (target, error) -> kS.get() * Math.signum(target);
    }

    /**
     * Returns a Controller that provides a constant output in the direction of the target.
     */
    static Controller staticTargetFeedforward(double kS) {
        return (target, error) -> kS * Math.signum(target);
    }

    /**
     * Provide a constant output proportional to the target.
     */
    static Controller proportionalFeedforward(Supplier<Double> kV) {
        return (target, error) -> kV.get() * target;
    }

    /**
     * Provide a constant output proportional to the target.
     */
    static Controller proportionalFeedforward(double kV) {
        return (target, error) -> kV * target;
    }

    /**
     * Provide a constant output proportional to the error.
     */
    static Controller proportional(Supplier<Double> kP) {
        return (target, error) -> kP.get() * error;
    }

    /**
     * Provide a constant output proportional to the error.
     */
    static Controller proportional(double kP) {
        return (target, error) -> kP * error;
    }

    /**
     * A controller that keeps track of time between updates.
     * This is useful for controllers that need to know the time between updates.
     */
    class TimedController implements Controller {
        double previousTime;
        double dt;

        @Override
        public double calculate(double target, double error) {
            if (previousTime == 0) {
                previousTime = System.nanoTime();
                return 0;
            }
            dt = (System.nanoTime() - previousTime) * 1e-9;
            previousTime = System.nanoTime();
            return 0;
        }

        @Override
        public void reset() {
            previousTime = 0;
            dt = 0;
        }
    }

    /**
     * Returns a Controller that provides an integral output based on the error over time.
     */
    static Controller integral(Supplier<Double> kI) {
        return new TimedController() {
            double integral = 0;

            @Override
            public double calculate(double target, double error) {
                super.calculate(target, error);
                integral += error * dt;
                return kI.get() * integral;
            }

            @Override
            public void reset() {
                super.reset();
                integral = 0;
            }
        };
    }

    /**
     * Returns a Controller that provides an integral output based on the error over time.
     */
    static Controller integral(
            Supplier<Double> kI, Supplier<Double> iZone, Supplier<Double> decay, Supplier<Double> maxI) {
        return new TimedController() {
            double integral = 0;

            @Override
            public double calculate(double target, double error) {
                super.calculate(target, error);

                if (Math.abs(error) < iZone.get()) {
                    integral *= decay.get();
                    integral += error * dt;
                    double cap = maxI.get();
                    integral = Math.max(-cap, Math.min(cap, integral));
                }
                return kI.get() * integral;
            }

            @Override
            public void reset() {
                super.reset();
                integral = 0;
            }
        };
    }

    /**
     * Returns a Controller that provides a derivative output based on the error over time.
     */
    static Controller derivative(Supplier<Double> kD) {
        return new TimedController() {
            double prevError = 0;
            boolean firstUpdate = true;

            @Override
            public double calculate(double target, double error) {
                super.calculate(target, error);

                if (dt < 1e-3 || firstUpdate) {
                    firstUpdate = false;
                    prevError = error;
                    return 0;
                }
                double output = kD.get() * (error - prevError) / dt;
                prevError = error;
                return output;
            }

            @Override
            public double calculate(double target, double error, double derivative) {
                return kD.get() * -derivative;
            }

            @Override
            public void reset() {
                super.reset();
                prevError = 0;
                firstUpdate = true;
            }
        };
    }

    /**
     * Returns a Controller that is the sum of this controller and another controller.
     */
    default Controller plus(Controller other) {
        return new Controller() {
            @Override
            public double calculate(double target, double error) {
                return Controller.this.calculate(target, error) + other.calculate(target, error);
            }

            @Override
            public double calculate(double target, double error, double derivative) {
                return Controller.this.calculate(target, error, derivative)
                        + other.calculate(target, error, derivative);
            }

            @Override
            public void reset() {
                Controller.this.reset();
                other.reset();
            }
        };
    }

    /**
     * Returns a Controller that is the difference of this controller and another controller.
     */
    default Controller minus(Controller other) {
        return new Controller() {
            @Override
            public double calculate(double target, double error) {
                return Controller.this.calculate(target, error) - other.calculate(target, error);
            }

            @Override
            public double calculate(double target, double error, double derivative) {
                return Controller.this.calculate(target, error, derivative)
                        - other.calculate(target, error, derivative);
            }

            @Override
            public void reset() {
                Controller.this.reset();
                other.reset();
            }
        };
    }

    /**
     * Returns a Controller that is the product of this controller and a scalar.
     */
    default Controller times(double scalar) {
        return new Controller() {
            @Override
            public double calculate(double target, double error) {
                return Controller.this.calculate(target, error) * scalar;
            }

            @Override
            public double calculate(double target, double error, double derivative) {
                return Controller.this.calculate(target, error, derivative) * scalar;
            }

            @Override
            public void reset() {
                Controller.this.reset();
            }
        };
    }
}
