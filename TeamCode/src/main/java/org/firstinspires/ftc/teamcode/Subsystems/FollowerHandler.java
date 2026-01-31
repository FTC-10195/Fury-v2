package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
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
    public static double brakePHeading = 8;
    public static double brakeDHeading = 1.2;
    public static double brakePDrive = 3;
    public static double brakeDDrive = 1;

    public static double brakePTranslational = 0.6;

    public static double pathingDTranslational  = 0.1;
    public static double pathingPHeading = 2.5;
    public static double pathingPTranslational = 0.3;
    
    public static Pose defaultPose = new Pose(72,72,Math.toRadians(0));
    public static Pose blueHumanPlayer = new Pose(136,8,Math.toRadians(0));
    public static Pose redHumanPlayer = new Pose(8,8,Math.toRadians(180));
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
               break;
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
        follower.setStartingPose(savedPose);
        follower.setPose(savedPose);
    }
    public void reset() {
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
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(brakePDrive, 0, brakeDDrive, Constants.followerConstants.getCoefficientsDrivePIDF().T, Constants.followerConstants.getCoefficientsDrivePIDF().F))
                .forwardZeroPowerAcceleration(Constants.followerConstants.getForwardZeroPowerAcceleration())
                .lateralZeroPowerAcceleration(Constants.followerConstants.getLateralZeroPowerAcceleration())
                .headingPIDFCoefficients(new PIDFCoefficients(brakePHeading,0,brakeDHeading,0.01))
                .translationalPIDFCoefficients(new PIDFCoefficients(brakePTranslational,0,brakeDTranslational,0.01)));
        follower.updateConstants();
    }
    public void setPathMode(){
        follower.setConstants( new FollowerConstants()
                .mass(Constants.followerConstants.getMass())
                .drivePIDFCoefficients(Constants.followerConstants.getCoefficientsDrivePIDF())
                .forwardZeroPowerAcceleration(Constants.followerConstants.getForwardZeroPowerAcceleration())
                .lateralZeroPowerAcceleration(Constants.followerConstants.getLateralZeroPowerAcceleration())
                .headingPIDFCoefficients(new PIDFCoefficients(pathingPHeading,0,0,0.01))
                .translationalPIDFCoefficients(new PIDFCoefficients(pathingPTranslational,0,pathingDTranslational,0.01)));
        follower.updateConstants();
    }

    private double vX = 0;
    private double vY = 0;
    private double prevXPos = 0;
    private double prevYPos = 0;
    private double deltaX = 0;
    private double deltaY = 0;
    private long prevTime = 0;
    private double deltaT = System.currentTimeMillis();

    public void update(){
        deltaX = follower.getPose().getX() - prevXPos;
        deltaY = follower.getPose().getY() - prevYPos;
        deltaT = (System.currentTimeMillis() - prevTime) / 1000.00;

        prevXPos = follower.getPose().getX();
        prevYPos = follower.getPose().getY();
        prevTime = System.currentTimeMillis();
        vX = deltaX / deltaT;
        vY = deltaY / deltaT;


        follower.update();
    }
    public void status(Telemetry telemetry){
        telemetry.addLine("FOLLOWER HANDLER -----------");
        telemetry.addData("FollowerHandlerPose",follower.getPose());
        telemetry.addData("Lock", locked);
        telemetry.addData("HoldPose",holdPose);

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());

        telemetry.addData("Saved",saved);
        if (savedPose != null) {
            telemetry.addData("saved pose", savedPose);
        }

        telemetry.addData("DeltaX",deltaX);
        telemetry.addData("DeltaY",deltaY);
        telemetry.addData("DeltaT",deltaT);
        telemetry.addData("vX",vX);
        telemetry.addData("vY",vY);
        telemetry.addData("velocityX",follower.getVelocity().getXComponent());
        telemetry.addData("velocityY",follower.getVelocity().getYComponent());
    }
    public double getVX(){
        return vX;
    }
    public double getVY(){
        return vY;
    }

}
