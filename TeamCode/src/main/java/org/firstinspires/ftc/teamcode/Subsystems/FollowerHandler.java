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
    
    public static Pose defaultPose = new Pose(72,72,Math.toRadians(0));
    public static Pose blueHumanPlayer = new Pose(132.89211422087746,10.021180030257188,Math.toRadians(0));
    public static Pose redHumanPlayer = new Pose(11.766546898638428,10.021180030257188,Math.toRadians(0));

    public static Pose savedPose;
    Follower follower;

    boolean locked = false;
    Pose holdPose;
    boolean saved = false;
    public void lock(){
        locked = true;
        holdPose = follower.getPose();
        setBrakeMode();

        follower.updateConstants();
        follower.holdPoint(holdPose);
    }
    public void unlock(){
        locked = false;
        follower.breakFollowing();
    }
    public void flipLock(){
       if (locked){
           unlock();
           return;
       }
       lock();
    }
    public boolean isLocked(){
        return locked;
    }

    public void initiate(HardwareMap hardwareMap){
        follower = Constants.createFollower(hardwareMap);
        setPathMode();
        load();
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
        saved = true;
        savedPose = follower.getPose();
    }
    public void load(){
        if (savedPose == null || !saved){
            savedPose = defaultPose;
        }
        follower.setPose(savedPose);
    }
    public void reset(){
        saved = false;
    }

    //For relocalization
    public void setStartingPose(Pose newPose){
        follower.setStartingPose(newPose);
        follower.setPose(newPose);
    }
    public Follower getFollower(){
        return follower;
    }
    public void setBrakeMode(){
        follower.setConstants( new FollowerConstants()
                .mass(Constants.followerConstants.getMass())
                .forwardZeroPowerAcceleration(Constants.followerConstants.getForwardZeroPowerAcceleration())
                .lateralZeroPowerAcceleration(Constants.followerConstants.getLateralZeroPowerAcceleration())
                .headingPIDFCoefficients(new PIDFCoefficients(brakePHeading,0,0,0.01))
                .translationalPIDFCoefficients(new PIDFCoefficients(brakePTranslational,0,brakeDTranslational,0.01)));
        follower.updateConstants();
    }
    public void setPathMode(){
        follower.setConstants( new FollowerConstants()
                .mass(Constants.followerConstants.getMass())
                .forwardZeroPowerAcceleration(Constants.followerConstants.getForwardZeroPowerAcceleration())
                .lateralZeroPowerAcceleration(Constants.followerConstants.getLateralZeroPowerAcceleration())
                .headingPIDFCoefficients(new PIDFCoefficients(pathingPHeading,0,0,0.01))
                .translationalPIDFCoefficients(new PIDFCoefficients(pathingPTranslational,0,pathingDTranslational,0.01)));
        follower.updateConstants();
    }
    public void update(){
        follower.update();
    }
    public void status(Telemetry telemetry){
        telemetry.addData("FollowerHandlerPose",follower.getPose());
        telemetry.addData("Lock", locked);
        telemetry.addData("HoldPose",holdPose);

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
    }
}
