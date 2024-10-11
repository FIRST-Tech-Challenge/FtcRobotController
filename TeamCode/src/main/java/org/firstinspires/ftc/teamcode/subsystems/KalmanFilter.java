package org.firstinspires.ftc.teamcode.subsystems;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {

    // State vector [x, y, yaw]
    private SimpleMatrix state;

    // Covariance matrix (estimate of state uncertainty)
    private SimpleMatrix covariance;

    // Process noise matrix (uncertainty in model prediction)
    private SimpleMatrix processNoise;

    // Measurement noise matrix (uncertainty in sensor measurements)
    private SimpleMatrix measurementNoise;

    // Transition matrix (models the evolution of the state over time)
    private SimpleMatrix transitionMatrix;

    // Measurement matrix (relates the state to the sensor measurements)
    private SimpleMatrix measurementMatrix;

    // Time step (in seconds)
    private double dt;

    public KalmanFilter(double timeStep) {
        this.dt = timeStep;

        // Initialize the state (3x1 vector: x, y, yaw)
        //This initializes the location and heading of the filter to 0
        state = new SimpleMatrix(3, 1);

        // Initialize the covariance matrix (3x3, initial uncertainty)
//        covariance = SimpleMatrix.identity(3).scale(0.5);
        //This is the initial confidence in the starting values
        //This is the uncertainty values of the initial values
        covariance = new SimpleMatrix(new double[][]{
                {0.5, 0, 0},//X pos
                {0, 0.5, 0},//Y pos
                {0, 0, 0.5} //H rotation
        });
        // Initialize the process noise (3x3)
        /*
        These values tell the filter to trust measurements more than predictions
        Increase in values --> MEASUREMENTS MORE and PREDICTIONS LESS
         */
        processNoise = new SimpleMatrix(new double[][]{
                {0.1, 0, 0},//X pos
                {0, 0.1, 0},//Y pos
                {0, 0, 10} //H rotation
        });

        // Initialize the measurement noise (3x3)
        /*
        These values tell the filter to trust predictions more than measurements
        Increase in values --> PREDICTIONS MORE and MEASUREMENTS LESS
         */
        measurementNoise = new SimpleMatrix(new double[][]{
                {0.01, 0, 0},//X pos
                {0, 0.01, 0},//Y pos
                {0, 0, 0.01} //H rotation
        });
        // Initialize the transition matrix (3x3)
        transitionMatrix = createTransitionMatrix(dt);

        // Initialize the measurement matrix (3x3, assuming sensors measure all states)
        measurementMatrix = SimpleMatrix.identity(3);
    }

    // Create a transition matrix that models constant velocity
    private SimpleMatrix createTransitionMatrix(double dt) {
        SimpleMatrix F = SimpleMatrix.identity(3);
        F.set(0, 1, dt);  // x = x + v_x * dt
        F.set(1, 2, dt);  // y = y + v_y * dt
        // Yaw rotation is constant for this simplified example
        return F;
    }

    // Prediction step
    public void predict() {
        // Predict the new state: x_k = F * x_(k-1)
        state = transitionMatrix.mult(state);

        // Update covariance: P_k = F * P_(k-1) * F^T + Q
        covariance = transitionMatrix.mult(covariance).mult(transitionMatrix.transpose()).plus(processNoise);
    }

    // Correction step using measurements from sensors
    public void correct(SimpleMatrix measurement) {
        // Calculate the Kalman Gain: K = P_k * H^T * (H * P_k * H^T + R)^-1
        SimpleMatrix Ht = measurementMatrix.transpose();
        SimpleMatrix S = measurementMatrix.mult(covariance).mult(Ht).plus(measurementNoise);
        SimpleMatrix K = covariance.mult(Ht).mult(S.invert());

        // Update state: x_k = x_k + K * (z_k - H * x_k)
        SimpleMatrix innovation = measurement.minus(measurementMatrix.mult(state));
        state = state.plus(K.mult(innovation));

        // Update covariance: P_k = (I - K * H) * P_k
        SimpleMatrix I = SimpleMatrix.identity(state.numRows());
        covariance = I.minus(K.mult(measurementMatrix)).mult(covariance);
    }

    // Method to update from three sensors
    public void updateFromSensors(double[] sensor1, double[] sensor2, double[] sensor3) {
        // Average sensor measurements (assuming all sensors measure x, y, and yaw)
        SimpleMatrix measurement = new SimpleMatrix(3, 1);
        for (int i = 0; i < 3; i++) {
            double avgMeasurement = (sensor1[i] + sensor2[i] + sensor3[i]) / 3.0;
            measurement.set(i, 0, avgMeasurement);
        }

        // Apply the correction step with the averaged measurement
        correct(measurement);
    }

    //sensors[sensor number][sensor data]
    public void updateFromSensors(double[][] sensors) {
        // Average sensor measurements (assuming all sensors measure x, y, and yaw)
        SimpleMatrix measurement = new SimpleMatrix(3, 1);
        for (int i = 0; i < 3; i++) {
            double total = 0;
            for(double[] sensorData : sensors){

                total += sensorData[i];
            }
            double avgMeasurement = total / sensors.length;
            measurement.set(i, 0, avgMeasurement);
        }

        // Apply the correction step with the averaged measurement
        correct(measurement);
    }

    // Getter for the current state (x, y, yaw)
    public SimpleMatrix getState() {
        return state;
    }

    //Updates time step value
    public void setTimeStep(double dt){
        this.dt = dt;
        this.transitionMatrix = createTransitionMatrix(dt);
    }

    public String getStateString(){
        return String.format("X: %.2f\nY: %.2f\nH: %.2f",
                state.get(0,0),
                state.get(1,0),
                state.get(2,0) * 2);
    }
}
