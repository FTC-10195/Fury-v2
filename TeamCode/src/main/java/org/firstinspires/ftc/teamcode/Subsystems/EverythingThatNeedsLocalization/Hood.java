package org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Hood {
    public enum States{
        ADJUST,
        RESTING,
        MANUAL
    }
    States state = States.RESTING;
    public static double downPos = 0.32;
    public static double upPos = 0.14;
    public static double downMax = downPos;
    public static double upMax = upPos;
    public static double defaultPos = (upPos + downPos) / 2;
    public boolean on = true;
    public static double maxDistance = 60;
    public static double a = 0;
    public static double b = 0.00366113;
    public static double c = 0.0822238;
    public static double offset = 0;
    public static double overMaxOffset = 0.14;
    Servo hood;
    public void setState(States state){
        this.state = state;
    }
    public double calculatePos(){
        //Relationship Here
        double distance = ShootingWhileMoving.getDistance();
        if (distance > maxDistance){
            return downPos - overMaxOffset;
        }
        return downPos - ((a * Math.pow(distance,2)) + (b * distance) + c);
    }
    public void initiate(HardwareMap hardwareMap){
        hood = hardwareMap.servo.get("hood");
    }
    public double manualPos = 0.5;
    double targetPos = defaultPos;
    public void update(){
        switch (state){
            case RESTING:
                targetPos = calculatePos();
                break;
            case ADJUST:
                targetPos = calculatePos();
                break;
            case MANUAL:
                targetPos = manualPos;
                break;
        }
        if (!on) {
            targetPos = defaultPos;
        }
        hood.setPosition(targetPos);
    }
    public void status(Telemetry telemetry){
        telemetry.addLine("Hood -----");
        telemetry.addData("Pos",hood.getPosition());
    }


}
