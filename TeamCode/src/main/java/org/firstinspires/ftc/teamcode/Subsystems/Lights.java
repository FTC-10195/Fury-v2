package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lights {
    public enum TeamColors {
        RED,
        BLUE,
        NONE
    }
    public enum Mode {
        TEAM,
        MOTIF,
        INTAKING,
        OVERRIDE_MODE,
        FOLLOWER_MODE
    }

    static TeamColors savedColor = TeamColors.RED;
    public static boolean saved = false;

    public void save() {
        saved = true;
        savedColor = teamColor;
    }


    public void reset() {
        savedColor = TeamColors.RED;
        saved = false;
    }

    Servo rgbIndicator;
    public static double blue = 0.6;
    public static double red = 0.28;
    public static double purple = .68;
    public static double green = .5;
    public static double yellow = .35; //Subsystem based/no team
    public static long motifSwitchWaitTime = 500;
    Timer motifSwitchTimer = new Timer();



    TeamColors teamColor = TeamColors.RED;
    Mode mode = Mode.TEAM;
    int sequence = 0;
    double color = 0;
    public void initiate(HardwareMap hardwareMap) {
        rgbIndicator = hardwareMap.get(Servo.class, "rgb");
    }
    LimeLight.BallColors ball = LimeLight.BallColors.NONE;
    LimeLight.BallColors[] motif = new LimeLight.BallColors[] {LimeLight.BallColors.P, LimeLight.BallColors.P, LimeLight.BallColors.G};

    public void setTeamColor(TeamColors newColor) {
        teamColor = newColor;
    }

    public TeamColors getTeamColor() {
        return teamColor;
    }
    public Mode getMode(){
        return mode;
    }
    public void setMode(Mode mode){
        this.mode = mode;
    }

    public void switchTeamColor() {
        saved = false;
        switch (teamColor) {
            case RED:
                teamColor = TeamColors.BLUE;
                break;
            case BLUE:
                teamColor = TeamColors.RED;
                break;
        }
    }
    public void setMotif(LimeLight.BallColors[] motif){
        this.motif = motif;
    }
    public void setBall(LimeLight.BallColors ball){
        this.ball = ball;
    }

    int motifIndex = 0;
    public void motifSequence(){
        if (motifIndex >= motif.length){
            motifIndex = 0;
        }
        switch (motif[motifIndex]){
            case G:
                color = green;
                break;
            case P:
                color = purple;
                break;
            case NONE:
                color = 0;
                break;
        }
        switch (sequence){
            case 0:
                motifSwitchTimer.setWait(motifSwitchWaitTime);
                sequence++;
                break;
            case 1:
                if (motifSwitchTimer.doneWaiting()){
                    sequence++;
                    if (motifIndex == motif.length - 1){
                        motifSwitchTimer.setWait(motifSwitchWaitTime * 2);
                    }else{
                        motifSwitchTimer.setWait(motifSwitchWaitTime/2);
                    }

                }
                break;
            case 2:
                color = 0;
                if (motifSwitchTimer.doneWaiting()){
                    motifIndex++;
                    sequence = 0;
                }

        }
    }
    public void intake(){
        switch (ball){
            case NONE:
                color = 0;
                break;
            case G:
                color = green;
                break;
            case P:
                color = purple;
        }
    }


    public void update(Telemetry telemetry) {
        telemetry.addLine("LIGHTS -----------");
        telemetry.addData("TeamColor", teamColor);
        telemetry.addData("LightsMode",mode);
        telemetry.addData("Saved Color",savedColor);
        telemetry.addData("Saved",saved);
        rgbIndicator.setPosition(color);
        if (saved){
            teamColor = savedColor;
        }
        switch (mode){
            case TEAM:
            switch (teamColor) {
                case BLUE:
                    color = blue;
                    break;
                case RED:
                    color = red;
                    break;
                case NONE:
                    color = 0;
                    break;
            }
            break;
            case MOTIF:
                motifSequence();
                break;
            case INTAKING:
                intake();
                break;
            case OVERRIDE_MODE:
                color = yellow;
                break;
            case FOLLOWER_MODE:
                color = green;
                break;
        }

    }
}
