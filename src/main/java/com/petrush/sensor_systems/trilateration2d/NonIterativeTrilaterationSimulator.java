package com.petrush.sensor_systems.trilateration2d;

import com.lemmingapex.trilateration.NonLinearLeastSquaresSolver;
import com.lemmingapex.trilateration.TrilaterationFunction;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer.Optimum;
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class NonIterativeTrilaterationSimulator {
    // input arguments
    public static final int NUMBER_OF_NODES = 300; // N
    public static final double FRACTION_OF_ANCHOR_NODES = 0.15; // f
    public static final double MAX_SIGNAL_ERROR = 0.45; // r -> value 0.15 means error can be at most 15%
    public static final double MAX_SENSOR_RANGE = 15; // R

    // specified constants
    public static final int NUMBER_OF_EXECUTIONS = 50; // # of executions on  different topologies

    // derived constants
    public static final long NUMBER_OF_ANCHOR_NODES = Math.round(NUMBER_OF_NODES * FRACTION_OF_ANCHOR_NODES);
    public static final long NUMBER_OF_NONANCHOR_NODES = NUMBER_OF_NODES - NUMBER_OF_ANCHOR_NODES;

    // computed instance variables
    private long numberOfLocatedNodes;
    private double averageLocationError;
    private double pctOfLocatedNodes;


    public static void main(String[] args) {
        List<NonIterativeTrilaterationSimulator> simulators = generateSimulatorsAndSimulate();
        calculateFinalStatisticsAndPrint(simulators);
    }

    private static List<NonIterativeTrilaterationSimulator> generateSimulatorsAndSimulate() {
        List<NonIterativeTrilaterationSimulator> simulators =
            Stream.generate(NonIterativeTrilaterationSimulator::new)
                .limit(NUMBER_OF_EXECUTIONS)
                .collect(Collectors.toList());
        simulators.forEach(NonIterativeTrilaterationSimulator::simulate);
        return simulators;
    }

    private static void calculateFinalStatisticsAndPrint(List<NonIterativeTrilaterationSimulator> simulators) {
        double averageLocalizationError;
        double numberOfLocatedNodes;
        double pctOfLocatedNodes;
        numberOfLocatedNodes = simulators.stream()
                .mapToLong(nonIterativeTrilaterationSimulator -> nonIterativeTrilaterationSimulator.numberOfLocatedNodes)
                .average().getAsDouble();
        averageLocalizationError = simulators.stream()
                .mapToDouble(nonIterativeTrilaterationSimulator -> nonIterativeTrilaterationSimulator.averageLocationError)
                .average().getAsDouble();
        pctOfLocatedNodes = simulators.stream()
                .mapToDouble(nonIterativeTrilaterationSimulator -> nonIterativeTrilaterationSimulator.pctOfLocatedNodes)
                .average().getAsDouble();

        System.out.println("Number of non-anchor nodes = " + NUMBER_OF_NONANCHOR_NODES);
        System.out.println("Number of located nodes = " + numberOfLocatedNodes);
        System.out.println("Fraction of located nodes = " + pctOfLocatedNodes + "%");
        System.out.println("Average localization error = " + averageLocalizationError);
    }

    private void simulate() {
        List<Node> nodes = createNodes();
        Map<Node, Map<Node, Double>> allDistances = calculateReachableDistances(nodes);
        locateAllNodesIfPossible(nodes, allDistances);
        calculateStatistics(nodes);
    }

    private static List<Node> createNodes() {
        List<Node> nodes = Stream.generate(Node::createRandomNode)
                .limit(NUMBER_OF_NODES)
                .collect(Collectors.toList());
        nodes.stream()
                .limit(NUMBER_OF_ANCHOR_NODES)
                .forEach(node -> node.isAnchor = true);
        return nodes;
    }

    private static Map<Node, Map<Node, Double>> calculateReachableDistances(List<Node> nodes) {
        return nodes.stream()
                .collect(Collectors.toMap(
                        node -> node,
                        node -> nodes.stream()
                                .filter(node2 -> !node.equals(node2)
                                        && distance(node, node2) <= MAX_SENSOR_RANGE)
                                .collect(
                                        Collectors.toMap(
                                                node2 -> node2,
                                                node2 -> distance(node, node2)
                                        ))
                ));
    }

    // returns euclidean distance of two nodes
    private static double distance(Node n1, Node n2) {
        return Math.sqrt((n2.y - n1.y) * (n2.y - n1.y) + (n2.x - n1.x) * (n2.x - n1.x));
    }

    // returns euclidean distance with uniformly distributed random error
    private static double distanceWithError(Node n1, Node n2) {
        double correctDistance = distance(n1, n2);
        int errorSign = Math.random() > 0.5 ? 1 : -1; // equally possible to have positive and negative error
        double error = errorSign * (MAX_SIGNAL_ERROR * Math.random());
        return (correctDistance + correctDistance * error);
    }

    private static void locateAllNodesIfPossible(List<Node> nodes, Map<Node, Map<Node, Double>> allDistances) {
        List<Node> nonAnchorNodes = nodes.stream()
                .filter(node -> !node.isAnchor)
                .collect(Collectors.toList());

        nonAnchorNodes.forEach(node -> locateNodeIfPossible(node, allDistances));

    }

    private static void locateNodeIfPossible(Node node, Map<Node, Map<Node, Double>> allDistances) {
        Map<Node, Double> neighborDistances = allDistances.get(node);
        Set<Node> neighbors = neighborDistances.keySet();
        List<Node> closestAnchorNeighbors = neighbors.stream()
                .filter(node1 -> node1.isAnchor)
                .sorted(Comparator.comparing(n -> distance(n, node)))
                .limit(3)
                .collect(Collectors.toList());
        if(closestAnchorNeighbors.size() >= 3) {
            locate(node, closestAnchorNeighbors);
        }
    }

    @SuppressWarnings("DuplicatedCode")
    private static void locate(Node locatedNode, List<Node> closestThreeAnchorNodes) {
        double[][] positions = getPositions(closestThreeAnchorNodes);
        double[] distances = getDistancesWithError(closestThreeAnchorNodes, locatedNode);
        NonLinearLeastSquaresSolver solver = new NonLinearLeastSquaresSolver(
                new TrilaterationFunction(positions, distances), new LevenbergMarquardtOptimizer());
        Optimum optimum = solver.solve();
        double[] predictedCoordinates = optimum.getPoint().toArray();
        locatedNode.predictedX = predictedCoordinates[0];
        locatedNode.predictedY = predictedCoordinates[1];
    }

    private static double[] getDistancesWithError(List<Node> anchorNodes, Node locatedNode) {
        return anchorNodes.stream()
                .mapToDouble(anchorNode -> distanceWithError(anchorNode, locatedNode))
                .toArray();
    }

    private static double[][] getPositions(List<Node> nodes) {
        Object[] objectArray = nodes.stream()
                .map(node -> new double[]{node.x, node.y}).toArray();
        double[][] positions = new double[nodes.size()][];
        for (int i = 0; i < nodes.size(); i++) {
            positions[i] = (double[]) objectArray[i];
        }
        return positions;
    }

    private void calculateStatistics(List<Node> nodes) {
        numberOfLocatedNodes = nodes.stream()
                .filter(node -> !node.isAnchor && node.predictedX != null)
                .count();
        averageLocationError = nodes.stream()
                .filter(node -> !node.isAnchor && node.predictedX != null)
                .mapToDouble(Node::calculatePredictionError)
                .average().orElseThrow(() -> new RuntimeException("No nodes were located"));
        pctOfLocatedNodes = numberOfLocatedNodes * 100.0 / NUMBER_OF_NONANCHOR_NODES;
    }
}
