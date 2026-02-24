package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Gate {
    public enum State{
        CLOSE,
        OPENNING,
        OPEN,
        CLOSING
    }
    public static double closePos = 1;
    public static double openPos = .7;
    public static long gateWaitTime = 800;
    public static long gateTransitionTime = 220;
    State currentState = State.CLOSE;
    Timer timer = new Timer();
    Servo servo;
    public void initiate(HardwareMap hardwareMap){
        servo = hardwareMap.servo.get("gate");
    }
    public void shoot(){
        timer.setWait(gateTransitionTime);
        currentState = State.OPENNING;
    }
    public State getState(){
        return currentState;
    }
    public boolean doneShooting(){
        return currentState == State.CLOSE;
    }
    public void update(){
        switch (currentState){
            case OPENNING:
                if (timer.doneWaiting()){
                    currentState = State.OPEN;
                    timer.setWait(gateWaitTime);
                }
                servo.setPosition(openPos);
                break;
            case OPEN:
                if (timer.doneWaiting()){
                    currentState = State.CLOSING;
                    timer.setWait(gateTransitionTime);
                }
                servo.setPosition(openPos);
                break;
            case CLOSING:
                if (timer.doneWaiting()){
                    currentState = State.CLOSE;
                }
                servo.setPosition(closePos);
                break;
            case CLOSE:
                servo.setPosition(closePos);
                break;
        }
    }
    public void status(Telemetry telemetry){
        telemetry.addLine("GATE ------");
        telemetry.addData("State",currentState);
    }

}
