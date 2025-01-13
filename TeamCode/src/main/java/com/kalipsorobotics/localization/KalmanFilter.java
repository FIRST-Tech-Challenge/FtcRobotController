package com.kalipsorobotics.localization;
import com.kalipsorobotics.localization.Matrix;

import org.opencv.core.Point;

public class KalmanFilter {

    private Point x; // State estimate (2D position)
    private double[][] P; // State covariance matrix (2x2)
    private double[][] F; // State transition matrix (2x2)
    private double[][] Q; // Process noise covariance matrix (2x2)
    private double[][] H; // Measurement matrix (2x2)
    private double R; // Measurement noise covariance (scalar)

    public KalmanFilter(double[][] F, double[][] Q, double[][] H, double R) {
        this.F = F;
        this.Q = Q;
        this.H = H;
        this.R = R;
        this.P = new double[2][2]; // 2x2 covariance matrix
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                P[i][j] = 0; // Initial covariance is zero
            }
        }
    }

    public void init(Point initialPosition) {
        this.x = initialPosition;
    }

    // Prediction step: Predict the state and covariance
    public void predict() {
        // State prediction: x = F * x
        double x1 = F[0][0] * x.x + F[0][1] * x.y;
        double y1 = F[1][0] * x.x + F[1][1] * x.y;
        this.x = new Point(x1, y1);

        // Covariance prediction: P = F * P * F' + Q
        double[][] Ptemp = new double[2][2];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                Ptemp[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j];
            }
        }

        double[][] Pnew = new double[2][2];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                Pnew[i][j] = Ptemp[i][0] * F[j][0] + Ptemp[i][1] * F[j][1] + Q[i][j];
            }
        }
        P = Pnew;
    }

    // Update step: Correct the prediction based on measurement
    public void update(Point z) {
        // Calculate the measurement residual (innovation): y = z - H * x
        double yx = z.x - (H[0][0] * x.x + H[0][1] * x.y);
        double yy = z.y - (H[1][0] * x.x + H[1][1] * x.y);
        Point y = new Point(yx, yy);

        // Calculate the innovation covariance: S = H * P * H' + R
        double S11 = H[0][0] * P[0][0] + H[0][1] * P[1][0];
        double S12 = H[0][0] * P[0][1] + H[0][1] * P[1][1];
        double S21 = H[1][0] * P[0][0] + H[1][1] * P[1][0];
        double S22 = H[1][0] * P[0][1] + H[1][1] * P[1][1];
        double S = S11 + S12 + S21 + S22 + R;  // Scalar innovation covariance

        // Compute Kalman gain: K = P * H' * (S)^(-1)
        double Kx = (P[0][0] * H[0][0] + P[0][1] * H[1][0]) / S;
        double Ky = (P[0][0] * H[0][1] + P[0][1] * H[1][1]) / S;

        // Update the state estimate: x = x + K * y
        this.x = new Point(x.x + Kx * y.x, x.y + Ky * y.y);

        // Update the state covariance: P = (I - K * H) * P
        double[][] KH = {{Kx * H[0][0] + Ky * H[1][0], Kx * H[0][1] + Ky * H[1][1]},
                {Kx * H[0][0] + Ky * H[1][0], Kx * H[0][1] + Ky * H[1][1]}};
        double[][] Pnew = new double[2][2];
        Pnew[0][0] = P[0][0] - KH[0][0] * P[0][0] - KH[0][1] * P[0][1];
        Pnew[0][1] = P[0][1] - KH[0][0] * P[0][1] - KH[0][1] * P[1][1];
        Pnew[1][0] = P[1][0] - KH[1][0] * P[0][0] - KH[1][1] * P[0][1];
        Pnew[1][1] = P[1][1] - KH[1][0] * P[0][1] - KH[1][1] * P[1][1];
        P = Pnew;
    }

    public Point cycle(Point point) {
        predict();
        update(point);
        return getState();
    }

    public Point getState() {
        return x;
    }
}