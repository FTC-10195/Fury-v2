package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Config
public class FollowerHandler {
    public static double brakeDTranslational = 0.1;
    public static double brakePHeading = 4;
    public static double brakePTranslational = 0.6;

    public static double pathingDTranslational  = 0.1;
    public static double pathingPHeading = 2.5;
    public static double pathingPTranslational = 0.3;

    public static double mass = 12;
    public static Pose defaultPose = new Pose(72,72,Math.toRadians(90));
    public static Pose blueHumanPlayer = new Pose(132.89211422087746,10.021180030257188,Math.toRadians(0));
    public static Pose redHumanPlayer = new Pose(11.766546898638428,10.021180030257188,Math.toRadians(0));


    private Pose pose;
    public static Pose savedPose;
    Follower follower;
    public void initiate(HardwareMap hardwareMap){
        follower = Constants.createFollower(hardwareMap);
        setPathMode();
        if (savedPose == null){
            savedPose = defaultPose;
            pose = savedPose;
        }
        follower.setStartingPose(pose);
    }
    public void forceRelocalize(Lights.TeamColors teamColor){
       switch (teamColor){
           case NONE:
               follower.setStartingPose(defaultPose);
               follower.setPose(defaultPose);
               break;
           case BLUE:
               follower.setStartingPose(blueHumanPlayer);
               follower.setPose(blueHumanPlayer);
               break;
           case RED:
               follower.setStartingPose(redHumanPlayer);
               follower.setPose(redHumanPlayer);
       }
    }
    public void save(){
        savedPose = follower.getPose();
    }
    public void load(){
        if (savedPose == null){
            savedPose = defaultPose;
        }
        follower.setPose(savedPose);
    }

    //For relocalization
    public void setPose(Pose newPose){
        pose = newPose;
        follower.setPose(pose);
    }
    public void setStartingPose(Pose newPose){
        pose = newPose;
        follower.setStartingPose(newPose);
        follower.setPose(newPose);
    }
    public Follower getFollower(){
        return follower;
    }
    public void setBrakeMode(){
        follower.setConstants( new FollowerConstants()
                .forwardZeroPowerAcceleration(-72.417)
                .lateralZeroPowerAcceleration(-83.947)
                .headingPIDFCoefficients(new PIDFCoefficients(brakePHeading,0,0,0.01))
                .translationalPIDFCoefficients(new PIDFCoefficients(brakePTranslational,0,brakeDTranslational,0.01))
                .mass(mass));
        follower.updateConstants();
    }
    public void setPathMode(){
        follower.setConstants( new FollowerConstants()
                .forwardZeroPowerAcceleration(-72.417)
                .lateralZeroPowerAcceleration(-83.947)
                .headingPIDFCoefficients(new PIDFCoefficients(pathingPHeading,0,0,0.01))
                .translationalPIDFCoefficients(new PIDFCoefficients(pathingPTranslational,0,pathingDTranslational,0.01))
                .mass(mass));
        follower.updateConstants();
    }
    public void update(){
        follower.update();
    }
    public void status(Telemetry telemetry){
        telemetry.addData("FollowerHandlerPose",pose);
    }
}
