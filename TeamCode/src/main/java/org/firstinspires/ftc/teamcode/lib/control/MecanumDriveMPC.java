package org.firstinspires.ftc.teamcode.lib.control;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumDriveModel;
import org.firstinspires.ftc.teamcode.team10515.Robot;

public class MecanumDriveMPC {
    public static final int HORIZON_STEP = 1000;
    public static final double dt = 0.001d;

    private static final SimpleMatrix TERMINATION_COST = new SimpleMatrix(6, 6, false, new double[] {
            100, 0, 0, 0, 0, 0,
            0, 100, 0, 0, 0, 0,
            0, 0, 100, 0, 0, 0,
            0, 0, 0, 100, 0, 0,
            0, 0, 0, 0, 100, 0,
            0, 0, 0, 0, 0, 100
    });

    private static final SimpleMatrix INTERMEDIARY_STATE_COST = new SimpleMatrix(6, 6, false, new double[] {
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
    });

    private static final SimpleMatrix INPUT_COST = new SimpleMatrix(4, 4, false, new double[] {
            1E-20, 0, 0, 0,
            0, 1E-20, 0, 0,
            0, 0, 1E-20, 0,
            0, 0, 0, 1E-20
    });

    private static final SimpleMatrix INPUT_CHANGE_COST = new SimpleMatrix(4, 4, false, new double[] {
            1E-20, 0, 0, 0,
            0, 1E-20, 0, 0,
            0, 0, 1E-20, 0,
            0, 0, 0, 1E-20
    });

    public MecanumDriveModel model;
    public SimpleMatrix[] P;
    public SimpleMatrix[] K;
    public SimpleMatrix state;
    public SimpleMatrix finalState;
    public SimpleMatrix lastInput;
    public int timeStep;

    public void initializeState() {
        //TODO: Simulate over policy lag
        state = Robot.getState();
        lastInput = Robot.getLastInput();
    }

    public void initializeState(Pose2d position) {
        state = new SimpleMatrix(6, 1, false, new double[] {
                position.getTranslation().x() / 100d, 0, position.getTranslation().y() / 100d, 0, position.getRotation().getRadians(), 0
        });

        lastInput = new SimpleMatrix(4, 1, false, new double[] {
                0, 0, 0, 0
        });
    }

    public MecanumDriveMPC(boolean selfInitialize) {
        if(selfInitialize) {
            initializeState();
        }
    }

    public static void main(String... args) {
        MecanumDriveModel model = new MecanumDriveModel(3000, 0.001, 15.75d, 0.315d, 0.315d * (0.1 * 0.1 + 0.032 * 0.032) / 2,
                0.315d * (3 * (0.1 * 0.1 + 0.032 * 0.032) + 0.05 * 0.05) / 12, 0.5613d,
                0.1d / 2, 13.7d, 2d, 12d, 0.187d, 9.2d,
                435 * 2 * Math.PI / 60d, 0.25d, 0.6d,
                7d * 0.0254, 7d * 0.0254, 7d * 0.0254, 7d * 0.0254);

        MecanumDriveMPC lqr = new MecanumDriveMPC(false);
        lqr.model = model;
        /*lqr.waypoints.add(new Waypoint(new DoubleMatrix(6, 1, new double[] {
                1, 0, 0, 0, 0, 0
        }), new DoubleMatrix(6, 6, new double[] {
                1, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0
        }), 0.5, 0.5));*/
        //lqr.initializeState(new Position2D(0, 0, new Angle(0d, AngularUnits.DEGREES)));
        //lqr.runLQR(new Position2D(1, 1, new Angle(90d, AngularUnits.DEGREES)));
        lqr.initializeState(new Pose2d(0d, 0d, new Rotation2d(0d, true)));
        lqr.runLQR(new Pose2d(100d, 100d, new Rotation2d(Math.toRadians(90d), true)));
        for(int i = 0; i < HORIZON_STEP; i++) {
            lqr.simulateSingleTimeStep(false);
            System.out.print(lqr.state);
            System.out.println(lqr.lastInput);
        }
    }

    /**
     * Run-to-position
     *
     * @param finalPosition
     */
    public void runLQR(Pose2d finalPosition) {
        SimpleMatrix finalState = new SimpleMatrix(6, 1, false, new double[] {
                finalPosition.getTranslation().x() / 100d, 0, finalPosition.getTranslation().y() / 100d, 0, finalPosition.getRotation().getRadians(), 0
        });

        runLQR(state, finalState, lastInput, Robot.isOptimizeInputChange());
    }

    public void runLQR(SimpleMatrix finalState) {
        runLQR(state, new SimpleMatrix(6, 1, false, new double[] {
                finalState.get(0) / 100, finalState.get(1) / 100, finalState.get(2) / 100,
                finalState.get(3) / 100, finalState.get(4), finalState.get(5)
        }), lastInput, Robot.isOptimizeInputChange());
    }

    public void runLQR(SimpleMatrix initialState, SimpleMatrix finalState, SimpleMatrix lastInput, boolean optimizeInputChange) {
        timeStep = 0;
        state = initialState;
        this.finalState = finalState;
        this.lastInput = lastInput;
        P = new SimpleMatrix[HORIZON_STEP];
        K = new SimpleMatrix[HORIZON_STEP - 1];
        P[P.length - 1] = getStateCost(HORIZON_STEP, optimizeInputChange);
        //expectedStates = new DoubleMatrix[HORIZON_STEP];
        //expectedStates[0] = initialState;

        SimpleMatrix AOriginal = model.stateTransitionMatrix(state, true);
        SimpleMatrix BOriginal = model.inputTransitionMatrix(state, false);
        SimpleMatrix A = getStateTransitionMatrix(AOriginal, BOriginal, optimizeInputChange);
        SimpleMatrix B = getInputTransitionMatrix(BOriginal, optimizeInputChange);

        if(!optimizeInputChange) {
            solveRiccatiEquation(HORIZON_STEP - 1, AOriginal, BOriginal, optimizeInputChange);
        } else {
            solveRiccatiEquation(HORIZON_STEP - 1, A, B, optimizeInputChange);
        }
    }

    /*public List<DoubleMatrix> lqrObstacles() {
        List<DoubleMatrix> disallowedTargets = new ArrayList<>();

        DoubleMatrix state = Robot.state;
        DoubleMatrix A = model.stateTransitionMatrix(state, true);
        DoubleMatrix B = model.inputTransitionMatrix(state, false);
        DoubleMatrix C = new DoubleMatrix(2, 6, new double[] {
                1, 0,
                0, 0,
                0, 1,
                0, 0,
                0, 0,
                0, 0
        });

        for(int i = 0; i < HORIZON_STEP - 1; i++) {
            int index = i;
            double timeStep = i * dt;
            DoubleMatrix temporaryState = state;
            DoubleMatrix gain = K[i].neg();
            DoubleMatrix feedbackTerm = A.sub(B.mmul(gain));
            DoubleMatrix G = B.mmul(gain).mul(timeStep).add(feedbackTerm.mmul(B.mmul(gain)).mul(timeStep * timeStep / 2));
            //Solve.pinv(feedbackTerm).mmul(DoubleMatrix.eye(6).addi(feedbackTerm.mul(dt / 2)).mmul(Solve.pinv(DoubleMatrix.eye(6).subi(feedbackTerm.mul(dt / 2))))).mmul(B).mmul(K[i]);
            DoubleMatrix inverse = Solve.pinv(C.mmul(G));
            obstacles.forEach(obstacle ->
                    obstacle.minkowskiSum(
                            C.neg().mmul(DoubleMatrix.eye(6).add(feedbackTerm.mul(timeStep))).mmul(temporaryState))
                            .forEach(position -> {
                                DoubleMatrix disallowedTarget = inverse.mmul(position).mul(100d);
                                if(disallowedTarget.get(0) >= 0 && disallowedTarget.get(0) <= 400 &&
                                        disallowedTarget.get(1) >= 0 && disallowedTarget.get(1) <= 400) {
                                    disallowedTargets.add(disallowedTarget);
                                }
                            }));
            //state = simulateSingleTimeStep(state, false);
            disallowedTargets.forEach(System.out::println);
        }

        return disallowedTargets;
    }*/

    /*public List<Vector2D> disallowedPoints() {
        return lqrObstacles().stream().map(position -> new Vector2D(position.get(0), position.get(1))).collect(Collectors.toList());
    }*/

    public SimpleMatrix simulate(boolean optimizeInputChange, double dt) {
        state = model.simulateDynamics(state, getOptimalInput(timeStep, state, lastInput, optimizeInputChange), dt);
        timeStep += (int)(dt / MecanumDriveMPC.dt);
        return state;
    }

    public SimpleMatrix simulateSingleTimeStep(boolean optimizeInputChange) {
        state = model.simulateDynamics(state, getOptimalInput(timeStep++, state, lastInput, optimizeInputChange));
        return state;
    }

    public SimpleMatrix simulateSingleTimeStep(SimpleMatrix state, boolean optimizeInputChange) {
        return model.simulateDynamics(state, getOptimalInput(timeStep++, state, lastInput, optimizeInputChange));
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, SimpleMatrix input, boolean optimizeInputChange) {
        if(optimizeInputChange) {
            if(timeStep < K.length) {
                lastInput = limitInput(input.plus(K[timeStep].mult(augmentState(state, input).minus(augmentState(finalState, new SimpleMatrix(4, 1))))));
            } else {
                lastInput = new SimpleMatrix(4, 1);
            }
        } else {
            if(timeStep < K.length) {
                lastInput = limitInput(K[timeStep].mult(state.minus(finalState)));
            } else {
                lastInput = new SimpleMatrix(4, 1);
            }
        }

        return lastInput;
    }

    public void solveRiccatiEquation(int timeStep, SimpleMatrix A, SimpleMatrix B, boolean optimizeInputChange) {
        if(timeStep < 1) {
            return;
        }

        SimpleMatrix Q = getStateCost(timeStep, optimizeInputChange);
        SimpleMatrix R = (optimizeInputChange ? INPUT_CHANGE_COST : INPUT_COST);
        SimpleMatrix inverse = R.plus(B.transpose().mult(P[timeStep].mult(B))).pseudoInverse();
        P[timeStep - 1] = Q.plus(A.transpose().mult(P[timeStep].mult(A))).minus(A.transpose().mult(P[timeStep].mult(B.mult(inverse).mult(B.transpose().mult(P[timeStep].mult(A))))));
        K[timeStep - 1] = new SimpleMatrix(4, optimizeInputChange ? 10 : 6).minus(inverse.mult(B.transpose()).mult(P[timeStep]).mult(A));
        solveRiccatiEquation(--timeStep, A, B, optimizeInputChange);
    }

    public static SimpleMatrix limitInput(SimpleMatrix control) {
        return new SimpleMatrix(4, 1, false, new double[] {
                control.get(0) > 1d ? 1d : control.get(0) < -1d ? -1d : control.get(0),
                control.get(1) > 1d ? 1d : control.get(1) < -1d ? -1d : control.get(1),
                control.get(2) > 1d ? 1d : control.get(2) < -1d ? -1d : control.get(2),
                control.get(3) > 1d ? 1d : control.get(3) < -1d ? -1d : control.get(3)
        });
    }

    public static double calculateCostToGo(int timeStep, SimpleMatrix state, SimpleMatrix input) {
        return state.transpose().mult(getStateCost(timeStep, false)).mult(state).plus(input.transpose().mult(INPUT_COST).mult(input)).get(0, 0);
    }

    private static SimpleMatrix augmentState(SimpleMatrix state, SimpleMatrix input) {
        return new SimpleMatrix(10, 1, false, new double[] {
                state.get(0),
                state.get(1),
                state.get(2),
                state.get(3),
                state.get(4),
                state.get(5),
                input.get(0),
                input.get(1),
                input.get(2),
                input.get(3)
        });
    }

    private static SimpleMatrix getStateCost(int timeStep, boolean optimizeInputChange) {
        SimpleMatrix Q = timeStep >= HORIZON_STEP - 1 ? TERMINATION_COST : INTERMEDIARY_STATE_COST;
        SimpleMatrix R = INPUT_COST;
        if(optimizeInputChange) {
            return new SimpleMatrix(10, 10, false, new double[]{
                    Q.get(0, 0), Q.get(1, 0), Q.get(2, 0), Q.get(3, 0), Q.get(4, 0), Q.get(5, 0), 0, 0, 0, 0,
                    Q.get(0, 1), Q.get(1, 1), Q.get(2, 1), Q.get(3, 1), Q.get(4, 1), Q.get(5, 1), 0, 0, 0, 0,
                    Q.get(0, 2), Q.get(1, 2), Q.get(2, 2), Q.get(3, 2), Q.get(4, 2), Q.get(5, 2), 0, 0, 0, 0,
                    Q.get(0, 3), Q.get(1, 3), Q.get(2, 3), Q.get(3, 3), Q.get(4, 3), Q.get(5, 3), 0, 0, 0, 0,
                    Q.get(0, 4), Q.get(1, 4), Q.get(2, 4), Q.get(3, 4), Q.get(4, 4), Q.get(5, 4), 0, 0, 0, 0,
                    Q.get(0, 5), Q.get(1, 5), Q.get(2, 5), Q.get(3, 5), Q.get(4, 5), Q.get(5, 5), 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, R.get(0, 0), R.get(1, 0), R.get(2, 0), R.get(3, 0),
                    0, 0, 0, 0, 0, 0, R.get(0, 1), R.get(1, 1), R.get(2, 1), R.get(3, 1),
                    0, 0, 0, 0, 0, 0, R.get(0, 2), R.get(1, 2), R.get(2, 2), R.get(3, 2),
                    0, 0, 0, 0, 0, 0, R.get(0, 3), R.get(1, 3), R.get(2, 3), R.get(3, 3)
            });
        }

        return Q;
    }

    private static SimpleMatrix getStateTransitionMatrix(SimpleMatrix A, SimpleMatrix B, boolean optimizeInputChange) {
        if(optimizeInputChange) {
            return new SimpleMatrix(10, 10, false, new double[]{
                    A.get(0, 0), A.get(1, 0), A.get(2, 0), A.get(3, 0), A.get(4, 0), A.get(5, 0), 0, 0, 0, 0,
                    A.get(0, 1), A.get(1, 1), A.get(2, 1), A.get(3, 1), A.get(4, 1), A.get(5, 1), 0, 0, 0, 0,
                    A.get(0, 2), A.get(1, 2), A.get(2, 2), A.get(3, 2), A.get(4, 2), A.get(5, 2), 0, 0, 0, 0,
                    A.get(0, 3), A.get(1, 3), A.get(2, 3), A.get(3, 3), A.get(4, 3), A.get(5, 3), 0, 0, 0, 0,
                    A.get(0, 4), A.get(1, 4), A.get(2, 4), A.get(3, 4), A.get(4, 4), A.get(5, 4), 0, 0, 0, 0,
                    A.get(0, 5), A.get(1, 5), A.get(2, 5), A.get(3, 5), A.get(4, 5), A.get(5, 5), 0, 0, 0, 0,
                    B.get(0, 0), B.get(1, 0), B.get(2, 0), B.get(3, 0), B.get(4, 0), B.get(5, 0), 1, 0, 0, 0,
                    B.get(0, 1), B.get(1, 1), B.get(2, 1), B.get(3, 1), B.get(4, 1), B.get(5, 1), 0, 1, 0, 0,
                    B.get(0, 2), B.get(1, 2), B.get(2, 2), B.get(3, 2), B.get(4, 2), B.get(5, 2), 0, 0, 1, 0,
                    B.get(0, 3), B.get(1, 3), B.get(2, 3), B.get(3, 3), B.get(4, 3), B.get(5, 3), 0, 0, 0, 1
            });
        }

        return A;
    }

    private static SimpleMatrix getInputTransitionMatrix(SimpleMatrix B, boolean optimizeInputChange) {
        if(optimizeInputChange) {
            return new SimpleMatrix(10, 4, false, new double[]{
                    B.get(0, 0), B.get(1, 0), B.get(2, 0), B.get(3, 0), B.get(4, 0), B.get(5, 0), 1, 0, 0, 0,
                    B.get(0, 1), B.get(1, 1), B.get(2, 1), B.get(3, 1), B.get(4, 1), B.get(5, 1), 0, 1, 0, 0,
                    B.get(0, 2), B.get(1, 2), B.get(2, 2), B.get(3, 2), B.get(4, 2), B.get(5, 2), 0, 0, 1, 0,
                    B.get(0, 3), B.get(1, 3), B.get(2, 3), B.get(3, 3), B.get(4, 3), B.get(5, 3), 0, 0, 0, 1
            });
        }

        return B;
    }
}
