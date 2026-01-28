package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret {
    public enum States{
        RESET, //Go to 0
        AIM, //Aim at goal
        MANUAL
    }
    Servo rightServo; //Dominant
    Servo leftServo;
    public static double startPos = .5;
    public static double maxDegrees = 320;
    public static double overridePos = .5;
    public static double maxPos = .85;
    public static double minPos = 0;
    public boolean shootWhileMoving = true;
    public static double degreesToTicks(double degrees){
        return startPos + (degrees/maxDegrees);
    }
    public static Pose redGoal = new Pose(144,144);
    public static Pose blueGoal = new Pose(0,144);
    private Pose goal = redGoal;
    private Pose robotPose = new Pose(0,0,Math.toRadians(0));
    private double target = 0;
    private States state = States.RESET;



    private double deltaX = 0;
    private double deltaY = 0;
    private double theta = 0;

    FollowerHandler followerHandler;
    Pose targetPose; //Calculated TargetPose for shooting while moving

    public void setFollowerHandler(FollowerHandler followerHandler){
        this.followerHandler = followerHandler;
    }

    public States getState(){
        return state;
    }
    public void setState(States state){
        this.state = state;
    }
    public void setGoal(Lights.TeamColors teamColor){
        switch (teamColor){
            case RED:
                goal = redGoal;
                break;
            case BLUE:
                goal = blueGoal;
                break;
        }
    }
    public static Pose getGoal(Lights.TeamColors teamColor){
        switch (teamColor){
            case RED:
                return  redGoal;
            case BLUE:
                return blueGoal;
        }
        return redGoal;
    }
    public Pose getGoal(){
        return goal;
    }
    double turretAngle;
    public double calculateHeading(){
        //RADIANS
        deltaX = targetPose.getX() - robotPose.getX();
        deltaY = targetPose.getY() - robotPose.getY();
        theta = Math.atan2(deltaY,deltaX);
        turretAngle = theta - robotPose.getHeading();
        if (turretAngle > Math.PI){
            turretAngle = -((2 * Math.PI) - turretAngle);
        }

        return turretAngle;
    }


    public void initiate(HardwareMap hardwareMap){
        rightServo = hardwareMap.servo.get("rt");
        leftServo = hardwareMap.servo.get("lt");
    }
    public void status(Telemetry telemetry){
        telemetry.addLine("TURRET -----------");
        telemetry.addData("turret target",target);
        telemetry.addData("turret state", state);
        telemetry.addData("Delta X", deltaX);
        telemetry.addData("Delta Y",deltaY);
        telemetry.addData("Theta Degrees", Math.toDegrees(theta));
        telemetry.addData("Robot heading degrees", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Turret Degrees", Math.toDegrees(calculateHeading()));
        ShootingWhileMoving.status(telemetry);
    }
    public void setOverride(double pos){
        overridePos = pos;
    }
    public void calculateOverrideAngle(Lights.TeamColors color ,double degrees){
        //default blue for auto
        if (color == Lights.TeamColors.RED){
            degrees = -degrees;
        }
        overridePos = degreesToTicks(degrees);
    }
    public void update(){
        if (followerHandler == null){
            return;
        }

        targetPose = goal;
        robotPose = followerHandler.getFollower().getPose();
        if (shootWhileMoving){
            targetPose = ShootingWhileMoving.calculateAimPose(goal,followerHandler);
        }

        switch (state){
            case RESET:
                target = startPos;
                break;
            case AIM:
                target = degreesToTicks(Math.toDegrees(calculateHeading()));
                break;
            case MANUAL:
                target = overridePos;
                break;
        }
        if (target > maxPos){
            target = maxPos;
        }
        if (target < minPos){
            target = minPos;
        }
        rightServo.setPosition(target);
        leftServo.setPosition(target);

    }



}
