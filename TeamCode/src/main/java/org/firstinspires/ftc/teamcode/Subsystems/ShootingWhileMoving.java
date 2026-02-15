package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class ShootingWhileMoving {
    //Shooting While moving
    //1. Constants
    public static long travelTimeNear = 500;
    public static double distanceNear = 65;
    public static long travelTimeFar = 900;
    public static double distanceFar = 140;
    public static double turretDampening = 0.9;
    public static double velocityThreshold = 5;

    //
    static double distance = 0; //Distance from goal (INCH)
    static double t = 0; //Either far zone time or near zone time (SEC)
    static double v = 0; //Calculated velocity based on far zone or near zone (INCH/SEC)
    static double travelTime = .1; //Estimated time for the ball to reach the goal
    static double deltaX = 0; //How off on the x axis the ball will be from the goal
    static double deltaY = 0;
    static Pose targetPose = new Pose(0,0,0); //Calculated aim pose to account for robots movement

    public static Pose calculateAimPose(Pose goal, FollowerHandler followerHandler){
        //Calculate Distance
        distance = goal.distanceFrom(followerHandler.getFollower().getPose());

        //Pick what data to use -> far zone or near zone and calculate average velocity
        t = (double) travelTimeNear / 1000.00;
        v = distanceNear / t;
        if (distance > Flywheel.farDistance){
            t = travelTimeFar / 1000.00;
            v = distanceFar / t;
        }

        //calculate travel time based on distance and velocity
        travelTime = distance / v;

        //Calculate Displacement based on robots x veloicty, y velocity, and time
        deltaX = followerHandler.getVX() * travelTime;
        deltaY = followerHandler.getVY() * travelTime;

        //Dampen deltaX and deltaY to account for turret lag
        deltaX *= turretDampening;
        deltaY *= turretDampening;

        //Calculate target pose based on deltaX and deltaY
        targetPose = new Pose(goal.getX() - deltaX, goal.getY() - deltaY);

        if (Math.abs(followerHandler.getVX()) < velocityThreshold && Math.abs(followerHandler.getVY()) < velocityThreshold){
            return goal;
        }

        return targetPose;

    }
    public static void status(Telemetry telemetry){
        telemetry.addLine("SHOOTING WHILE MOVING -----");
        telemetry.addData("Distance",distance);
        telemetry.addData("t",t);
        telemetry.addData("v",v);
        telemetry.addData("travelTime",travelTime);
        telemetry.addData("DeltaX",deltaX);
        telemetry.addData("DeltaY",deltaY);
        telemetry.addData("TargetPose",targetPose);
    }
    public static void updateFTCDashboard(TelemetryPacket telemetryPacket){
        telemetryPacket.put("Travel Time",travelTime);
        telemetryPacket.put("Delta X",deltaX);
        telemetryPacket.put("Delta Y",deltaY);
        telemetryPacket.put("Velocity",v);
        telemetryPacket.put("Target X",targetPose.getX());
        telemetryPacket.put("Target Y",targetPose.getY());



    }


}
