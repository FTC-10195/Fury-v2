package org.firstinspires.ftc.teamcode.Subsystems.Webcam;

import java.util.ArrayList;

public class Cluster {
    ArrayList<Double> cluster;
    public Cluster(ArrayList<Double> cluster){
        this.cluster = cluster;
    }
    public ArrayList<Double> get(){
        return cluster;
    }
    public int size(){
        return cluster.size();
    }
    public double getCenter(){
        return getCenterOfCluster(get());
    }
    public static double getMinOfCluster(ArrayList<Double> cluster){
        double min = 1000;
        for (int i = 0; i < cluster.size(); i ++){
            if (cluster.get(i) < min){
                min = cluster.get(i);
            }
        }
        return min;
    }
    public static double getMaxOfCluster(ArrayList<Double> cluster){
        double max = 0;
        for (int i = 0; i < cluster.size(); i ++){
            if (cluster.get(i) > max){
                max = cluster.get(i);
            }
        }
        return max;
    }
    public static Cluster createCluster(ArrayList<Double> balls, double start, double end){
        ArrayList<Double> newBallsList = new ArrayList<>();
        for (int i = 0; i < balls.size(); i ++){
            if (balls.get(i) >= start && balls.get(i) <= end){
                newBallsList.add(balls.get(i));
            }
        }
        return new Cluster(newBallsList);
    }
    public static ArrayList<Cluster> createClusters(ArrayList<Double> balls){
        ArrayList<Cluster> newClustersList = new ArrayList<>();
        for (int i = 0; i < balls.size(); i ++){
            newClustersList.add(createCluster(balls,balls.get(i),balls.get(i) + Webcam.intakeWidth));
        }
        return newClustersList;
    }
    public static double getCenterOfCluster(ArrayList<Double> cluster){
        if (cluster.size() == 1){
            return cluster.get(0);
        }
        return (getMaxOfCluster(cluster) + getMinOfCluster(cluster))/2;
    }
    public static Cluster getClosestCluster(ArrayList<Cluster> clusters){
        int cameraCenter = Webcam.cameraWidth / 2;
        Cluster currentCluster = clusters.get(0);
        double distance = 1000;
        for (int i = 0; i < clusters.size(); i ++){
            if (Math.abs(clusters.get(i).getCenter() - cameraCenter) < distance ){
                distance = Math.abs(clusters.get(i).getCenter() - cameraCenter);
                currentCluster = clusters.get(i);
            }
        }
        return currentCluster;
    }
    public static Cluster getBiggestCluster(ArrayList<Cluster> clusters){
        int size = 0;
        Cluster currentCluster = clusters.get(0);
        for (int i = 0; i < clusters.size(); i ++){
            if (clusters.get(i).size() > size){
                size = clusters.get(i).size();
                currentCluster = clusters.get(i);
            }
        }
        return currentCluster;
    }
}
