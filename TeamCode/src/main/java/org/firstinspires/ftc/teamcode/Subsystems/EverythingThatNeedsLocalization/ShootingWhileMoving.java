package org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;

@Config
public class ShootingWhileMoving {
    public static boolean shootWhileMoving = true;
    //Shooting While moving
    //1. Constants
    public static long travelTimeNear = 500;
    public static double distanceNear = 65;
    public static long travelTimeFar = 900;
    public static double distanceFar = 140;
    public static double turretDampening = 0.9;
    public static double velocityThreshold = 5;
    public static Pose redGoal = new Pose(144,144);
    public static Pose blueGoal = new Pose(0,144);



    private static Pose goal = redGoal;
    private static Pose robotPose = FollowerHandler.defaultPose;
    private static double distance = 0; //Distance from goal (INCH)
    private static double t = 0; //Either far zone time or near zone time (SEC)
    private static double v = 0; //Calculated velocity based on far zone or near zone (INCH/SEC)
    private static double travelTime = .1; //Estimated time for the ball to reach the goal
    private static double deltaX = 0; //How off on the x axis the ball will be from the goal
    private static double deltaY = 0;
    private static boolean returnGoal = false;
    static Pose targetPose = new Pose(0,0,0); //Calculated aim pose to account for robots movement
    public static Pose getGoal(Lights.TeamColors teamColor){
        switch (teamColor){
            case RED:
                return  redGoal;
            case BLUE:
                return blueGoal;
        }
        return redGoal;
    }
    public static Pose getGoal(){
        return goal;
    }

    public static double getDistance(){
        return distance;
    }
    public static Pose getRobotPose(){
        return robotPose;
    }


    //Run every loop ONCE
    public static void update(FollowerHandler followerHandler, Lights.TeamColors teamColor){
        robotPose = followerHandler.getFollower().getPose();
        goal = getGoal(teamColor);
        returnGoal = false;
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
            returnGoal = true;
        }
        //Checks for teleporting aka relocalization - specifically when bot is not moving or moving slowly
        if (Math.abs(followerHandler.getFollower().getVelocity().getMagnitude()) < LimeLight.velocityCountsAsMovingThreshold){
            returnGoal = true;
        }
        //If its turned off then dont use
        if (!shootWhileMoving){
            returnGoal = true;
        }
    }

    public static Pose getAimPose(){
        if (returnGoal){
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
