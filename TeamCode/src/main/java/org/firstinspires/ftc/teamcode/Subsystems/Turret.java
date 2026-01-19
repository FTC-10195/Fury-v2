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
    public static double maxDegrees = 315; //-1100 to 110
    public static double overridePos = .75;
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


    public States getState(){
        return state;
    }
    public void setState(States state){
        this.state = state;
    }
    public void setPose(Pose pose){
        robotPose = pose;
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
        deltaX = goal.getX() - robotPose.getX();
        deltaY = goal.getY() - robotPose.getY();
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
    }
    public void setOverride(double pos){
        overridePos = pos;
    }
    public void update(){

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
        if (target > 1){
            target = 1;
        }
        if (target < 0){
            target = 0;
        }
        rightServo.setPosition(target);
        leftServo.setPosition(target);

    }
}
