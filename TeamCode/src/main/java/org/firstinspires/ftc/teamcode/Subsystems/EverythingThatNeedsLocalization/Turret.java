package org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;

@Config
public class Turret {
    public enum States{
        RESET, //Go to 0
        AIM, //Aim at goal
        MANUAL
    }
    Servo rightServo; //Dominant
    Servo leftServo;
    public boolean on = true;
    public static double startPos = .51;
    public static double maxDegrees = 327.27;
    public static double overridePos = .51;
    public static double maxPos = .85;
    public static double minPos = 0;
    public static double degreesToTicks(double degrees){
        return startPos + (degrees/maxDegrees);
    }

    public static double manualGain = .01;
    public double manualOffset = 0;
    private double target = startPos;
    private States state = States.RESET;



    private double deltaX = 0;
    private double deltaY = 0;
    private double theta = 0;

    public void add(){
        manualOffset += manualGain;
    }
    public void sub(){
        manualOffset -= manualGain;
    }

    Pose targetPose; //Calculated TargetPose for shooting while moving


    public States getState(){
        return state;
    }
    public void setState(States state){
        this.state = state;
    }
    double turretAngle;
    public double calculateHeading(){
        //RADIANS
        deltaX = targetPose.getX() - ShootingWhileMoving.getRobotPose().getX();
        deltaY = targetPose.getY() - ShootingWhileMoving.getRobotPose().getY();
        theta = Math.atan2(deltaY,deltaX);
        turretAngle = theta - ShootingWhileMoving.getRobotPose().getHeading();
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
        telemetry.addData("On",on);
        telemetry.addData("Theta Degrees", Math.toDegrees(theta));
        telemetry.addData("Robot heading degrees", Math.toDegrees(ShootingWhileMoving.getRobotPose().getHeading()));
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
        targetPose = ShootingWhileMoving.getAimPose();


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
        if (!on){
            target = startPos;
        }
        target += manualOffset;
        rightServo.setPosition(target);
        leftServo.setPosition(target);

    }
    public void updateFTCDashboard(TelemetryPacket telemetryPacket){
        ShootingWhileMoving.updateFTCDashboard(telemetryPacket);
    }



}
