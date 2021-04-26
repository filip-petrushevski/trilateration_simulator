package com.petrush.sensor_systems.trilateration3d;

public class Node {
    public static final double MAP_DIMENSION = 100; // L

    public double x;
    public double y;
    public double z;
    public boolean isAnchor;
    public Double predictedX;
    public Double predictedY;
    public Double predictedZ;
    public int nonRelevanceLevel; // used only in Iterative Trilateration

    public static Node createRandomNode() {
        Node node = new Node();
        node.x = Math.random() * MAP_DIMENSION;
        node.y = Math.random() * MAP_DIMENSION;
        node.z = Math.random() * MAP_DIMENSION;
        node.isAnchor = false;

        return node;
    }
    
    public double calculatePredictionError() {
        if (predictedX == null) {
            throw new RuntimeException("Node location was never predicted");
        }
        return Math.sqrt((predictedY - y) * (predictedY - y) + (predictedX - x) * (predictedX - x) + (predictedZ - z) * (predictedZ - z));
    }

    @Override
    public String toString() {
        return "Node[" +
                "x=" + x +
                ", y=" + y +
                ", z=" + z +
                (isAnchor ? ", Anchor" : "") +
                ']';
    }


}
