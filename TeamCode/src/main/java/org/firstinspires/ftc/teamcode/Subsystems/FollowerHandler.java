package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Config
public class FollowerHandler {
    public enum State{
        PARK,
        BRAKE,
        RESTING
    }
    public State state = State.RESTING;
    public void setState(State state){
        this.state = state;
    }
    public State getState(){
        return state;
    }
    public static double brakeDTranslational = 0.1;
    public static double brakePHeading = 8;
    public static double brakeDHeading = 1.2;
    public static double brakePDrive = 3;
    public static double brakeDDrive = 1;

    public static double brakePTranslational = 0.6;
    
    public static Pose defaultPose = new Pose(72,72,Math.toRadians(0));
    public static Pose blueHumanPlayer = new Pose(136,8,Math.toRadians(0));
    public static Pose redHumanPlayer = new Pose(8,8,Math.toRadians(180));
    public static Pose redPark = new Pose(41.682,32.909390444810555,Math.toRadians(180));
    public static Pose bluePark = new Pose(102.5820428336079,32.909390444810555,Math.toRadians(0));

    public static Pose savedPose;
    Follower follower;

    Pose holdPose;
    Pose parkPose;
    PathChain park;
    boolean saved = false;
    public void autoPark(Lights.TeamColors color){
        state = State.PARK;
        switch (color){
            case RED:
                parkPose = redPark;
                break;
            default:
                parkPose = bluePark;
                break;
        }
        park = follower.pathBuilder().addPath(
                        new BezierLine(
                                follower.getPose(),
                                parkPose
                        )
                )
                .setLinearHeadingInterpolation(follower.getHeading(), parkPose.getHeading())
                        .build();


        setPathMode();
        follower.updateConstants();

        follower.followPath(park);

    }
    public void rest(){
        state = State.RESTING;
        follower.breakFollowing();
        setPathMode();
    }
    public void flipLock(Lights light){
       if (state == State.BRAKE){
           light.setMode(Lights.Mode.TEAM);
           rest();
           return;
       }
        light.setMode(Lights.Mode.FOLLOWER_MODE);
        state = State.BRAKE;
        holdPose = follower.getPose();
        setBrakeMode();

        follower.updateConstants();
        follower.holdPoint(holdPose);
    }
    public void flipPark(Lights light){
        if (state == State.PARK || state == State.BRAKE){
            light.setMode(Lights.Mode.TEAM);
            rest();
            return;
        }
        light.setMode(Lights.Mode.FOLLOWER_MODE);
        autoPark(light.getTeamColor());
    }
    public boolean isLocked(){
    return state == State.BRAKE;
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
    public void end() {
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
        follower.setConstants(Constants.followerConstants);
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
        telemetry.addData("State",state);
        telemetry.addData("Lock", state == State.BRAKE);
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
