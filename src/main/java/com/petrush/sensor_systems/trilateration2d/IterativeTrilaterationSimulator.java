package com.petrush.sensor_systems.trilateration2d;

import com.lemmingapex.trilateration.NonLinearLeastSquaresSolver;
import com.lemmingapex.trilateration.TrilaterationFunction;
import com.petrush.sensor_systems.trilateration.Heuristic;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer.Optimum;
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer;

import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class IterativeTrilaterationSimulator {
    // input arguments
    public static final Heuristic HEURISTIC = Heuristic.CLOSEST_ANCHOR_NEIGHBOR;
    public static final int NUMBER_OF_NODES = 300; // N
    public static final double FRACTION_OF_ANCHOR_NODES = 0.15; // f
    public static final double MAX_SIGNAL_ERROR = 0.15; // r -> value 0.15 means error can be at most 15%
    public static final double MAX_SENSOR_RANGE = 15; // R

    public static final int NUMBER_OF_EXECUTIONS = 50; // # of executions on  different topologies

    // derived constants
    public static final long NUMBER_OF_ANCHOR_NODES = Math.round(NUMBER_OF_NODES * FRACTION_OF_ANCHOR_NODES);
    public static final long NUMBER_OF_NON_ANCHOR_NODES = NUMBER_OF_NODES - NUMBER_OF_ANCHOR_NODES;

    // computed instance variables
    private long numberOfLocatedNodes;
    private double averageLocationError;
    private double pctOfLocatedNodes;


    public static void main(String[] args) {
        List<IterativeTrilaterationSimulator> simulators = generateSimulatorsAndSimulate();
        calculateFinalStatisticsAndPrint(simulators);
    }

    private static List<IterativeTrilaterationSimulator> generateSimulatorsAndSimulate() {
        List<IterativeTrilaterationSimulator> simulators =
                Stream.generate(IterativeTrilaterationSimulator::new)
                        .limit(NUMBER_OF_EXECUTIONS)
                        .collect(Collectors.toList());
        simulators.forEach(IterativeTrilaterationSimulator::simulate);
        return simulators;
    }

    private static void calculateFinalStatisticsAndPrint(List<IterativeTrilaterationSimulator> simulators) {
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

        System.out.println("Number of non-anchor nodes = " + NUMBER_OF_NON_ANCHOR_NODES);
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
                .forEach(node -> {
                    node.isAnchor = true;
                    node.predictedX = node.x;
                    node.predictedY = node.y;
                    node.nonRelevanceLevel = 0;
                });
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

    // returns predicted euclidean distance of two nodes
    private static double predictedDistance(Node n1, Node locatedNode) {
        return Math.sqrt((locatedNode.y - n1.predictedY) * (locatedNode.y - n1.predictedY) +
                (locatedNode.x - n1.predictedX) * (locatedNode.x - n1.predictedX));
    }

    // returns euclidean distance with uniformly distributed random error
    private static double distanceWithError(Node n1, Node n2) {
        double predictedDistance = predictedDistance(n1, n2);
        int errorSign = Math.random() > 0.5 ? 1 : -1; // equally possible to have positive and negative error
        double error = errorSign * (MAX_SIGNAL_ERROR * Math.random());
        return (predictedDistance + predictedDistance * error);
    }

    private static void locateAllNodesIfPossible(List<Node> nodes, Map<Node, Map<Node, Double>> allDistances) {
        int nonAnchorNodesSize = -1;
        // iterate
        while (true) {
            List<Node> nonAnchorNodes = nodes.stream()
                    .filter(node -> node.predictedX == null)
                    .collect(Collectors.toList());
            if (nonAnchorNodes.size() == nonAnchorNodesSize) {
                // if no new nodes were located with the last iteration
                break;
            }
            nonAnchorNodesSize = nonAnchorNodes.size();
            nonAnchorNodes.forEach(node -> locateNodeIfPossible(node, allDistances));
        }

    }

    private static void locateNodeIfPossible(Node node, Map<Node, Map<Node, Double>> allDistances) {
        Map<Node, Double> neighborDistances = allDistances.get(node);
        Set<Node> neighbors = neighborDistances.keySet();
        List<Node> closestAnchorNeighbors = getClosestAnchorNeighbors(node, neighbors);
        if (closestAnchorNeighbors.size() >= 3) {
            locate(node, closestAnchorNeighbors);
        }
    }

    private static List<Node> getClosestAnchorNeighbors(Node node, Set<Node> neighbors) {
        Comparator<Object> comparator;
        if (Heuristic.CLOSEST_ANCHOR_NEIGHBOR.equals(HEURISTIC)) {
            comparator = Comparator.comparing(n -> distance((Node) n, node));
        } else if (Heuristic.MOST_RELEVANT_ANCHOR_NEIGHBOR.equals(HEURISTIC)) {
            comparator = Comparator.comparing(n -> ((Node)n).nonRelevanceLevel)
                                    .thenComparing(n -> distance((Node) n, node));
        } else {
            throw new RuntimeException("Specified heuristic not implemented");
        }

        return neighbors.stream()
                .filter(node1 -> node1.predictedX != null)
                .sorted(comparator)
                .limit(3)
                .collect(Collectors.toList());
    }

    private static void locate(Node locatedNode, List<Node> closestThreeAnchorNodes) {
        double[][] positions = getPositions(closestThreeAnchorNodes);
        double[] distances = getDistancesWithError(closestThreeAnchorNodes, locatedNode);
        NonLinearLeastSquaresSolver solver = new NonLinearLeastSquaresSolver(new TrilaterationFunction(positions, distances), new LevenbergMarquardtOptimizer());
        Optimum optimum = solver.solve();
        double[] predictedCoordinates = optimum.getPoint().toArray();
        locatedNode.predictedX = predictedCoordinates[0];
        locatedNode.predictedY = predictedCoordinates[1];
        locatedNode.nonRelevanceLevel = 1 + closestThreeAnchorNodes.stream()
                .mapToInt(anchorNode -> anchorNode.nonRelevanceLevel)
                .sum();
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
        pctOfLocatedNodes = numberOfLocatedNodes * 100.0 / NUMBER_OF_NON_ANCHOR_NODES;
    }
}
