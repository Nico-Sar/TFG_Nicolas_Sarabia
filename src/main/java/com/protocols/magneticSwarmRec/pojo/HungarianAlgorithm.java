package com.protocols.magneticSwarmRec.pojo;

import java.util.Arrays;

public class HungarianAlgorithm {

    private final double[][] costMatrix;
    private final int rows, cols, dim;
    private final double[] labelByWorker, labelByJob;
    private final int[] minSlackWorkerByJob;
    private final double[] minSlackValueByJob;
    private final int[] matchJobByWorker, matchWorkerByJob;
    private final int[] parentWorkerByCommittedJob;
    private final boolean[] committedWorkers;

    public HungarianAlgorithm(double[][] costMatrix) {
        this.dim = Math.max(costMatrix.length, costMatrix[0].length);
        this.costMatrix = new double[this.dim][this.dim];
        this.rows = costMatrix.length;
        this.cols = costMatrix[0].length;

        for (int w = 0; w < this.rows; w++) {
            if (costMatrix[w].length != this.cols) throw new IllegalArgumentException("Irregular cost matrix");
            System.arraycopy(costMatrix[w], 0, this.costMatrix[w], 0, this.cols);
        }

        labelByWorker = new double[dim];
        labelByJob = new double[dim];
        minSlackWorkerByJob = new int[dim];
        minSlackValueByJob = new double[dim];
        committedWorkers = new boolean[dim];
        parentWorkerByCommittedJob = new int[dim];
        matchJobByWorker = new int[dim];
        matchWorkerByJob = new int[dim];

        Arrays.fill(matchJobByWorker, -1);
        Arrays.fill(matchWorkerByJob, -1);
    }

    public int[] execute() {
        reduce();
        computeInitialFeasibleSolution();
        greedyMatch();

        int w = fetchUnmatchedWorker();
        while (w < dim) {
            initializePhase(w);
            executePhase();
            w = fetchUnmatchedWorker();
        }

        int[] result = new int[rows];
        for (int i = 0; i < rows; i++) {
            result[i] = matchJobByWorker[i];
        }
        return result;
    }

    private void reduce() {
        for (int w = 0; w < dim; w++) {
            double min = Double.POSITIVE_INFINITY;
            for (int j = 0; j < dim; j++) min = Math.min(min, costMatrix[w][j]);
            for (int j = 0; j < dim; j++) costMatrix[w][j] -= min;
        }
        for (int j = 0; j < dim; j++) {
            double min = Double.POSITIVE_INFINITY;
            for (int w = 0; w < dim; w++) min = Math.min(min, costMatrix[w][j]);
            for (int w = 0; w < dim; w++) costMatrix[w][j] -= min;
        }
    }

    private void computeInitialFeasibleSolution() {
        for (int j = 0; j < dim; j++) {
            labelByJob[j] = Double.POSITIVE_INFINITY;
        }
        for (int w = 0; w < dim; w++) {
            for (int j = 0; j < dim; j++) {
                labelByJob[j] = Math.min(labelByJob[j], costMatrix[w][j]);
            }
        }
    }

    private void greedyMatch() {
        for (int w = 0; w < dim; w++) {
            for (int j = 0; j < dim; j++) {
                if (matchJobByWorker[w] == -1 && matchWorkerByJob[j] == -1 &&
                        costMatrix[w][j] - labelByWorker[w] - labelByJob[j] == 0) {
                    match(w, j);
                }
            }
        }
    }

    private void initializePhase(int w) {
        Arrays.fill(committedWorkers, false);
        Arrays.fill(parentWorkerByCommittedJob, -1);
        committedWorkers[w] = true;
        for (int j = 0; j < dim; j++) {
            minSlackValueByJob[j] = costMatrix[w][j] - labelByWorker[w] - labelByJob[j];
            minSlackWorkerByJob[j] = w;
        }
    }

    private void executePhase() {
        while (true) {
            int minSlackWorker = -1, minSlackJob = -1;
            double minSlackValue = Double.POSITIVE_INFINITY;
            for (int j = 0; j < dim; j++) {
                if (parentWorkerByCommittedJob[j] == -1) {
                    if (minSlackValueByJob[j] < minSlackValue) {
                        minSlackValue = minSlackValueByJob[j];
                        minSlackWorker = minSlackWorkerByJob[j];
                        minSlackJob = j;
                    }
                }
            }

            if (minSlackValue > 0) updateLabeling(minSlackValue);

            parentWorkerByCommittedJob[minSlackJob] = minSlackWorker;
            if (matchWorkerByJob[minSlackJob] == -1) {
                int committedJob = minSlackJob;
                int parentWorker = parentWorkerByCommittedJob[committedJob];
                while (true) {
                    int temp = matchJobByWorker[parentWorker];
                    match(parentWorker, committedJob);
                    committedJob = temp;
                    if (committedJob == -1) break;
                    parentWorker = parentWorkerByCommittedJob[committedJob];
                }
                return;
            } else {
                int worker = matchWorkerByJob[minSlackJob];
                committedWorkers[worker] = true;
                for (int j = 0; j < dim; j++) {
                    if (parentWorkerByCommittedJob[j] == -1) {
                        double slack = costMatrix[worker][j] - labelByWorker[worker] - labelByJob[j];
                        if (minSlackValueByJob[j] > slack) {
                            minSlackValueByJob[j] = slack;
                            minSlackWorkerByJob[j] = worker;
                        }
                    }
                }
            }
        }
    }

    private void updateLabeling(double slack) {
        for (int w = 0; w < dim; w++) {
            if (committedWorkers[w]) labelByWorker[w] += slack;
        }
        for (int j = 0; j < dim; j++) {
            if (parentWorkerByCommittedJob[j] != -1) labelByJob[j] -= slack;
            else minSlackValueByJob[j] -= slack;
        }
    }

    private int fetchUnmatchedWorker() {
        for (int w = 0; w < dim; w++) {
            if (matchJobByWorker[w] == -1) return w;
        }
        return dim;
    }

    private void match(int w, int j) {
        matchJobByWorker[w] = j;
        matchWorkerByJob[j] = w;
    }
}
