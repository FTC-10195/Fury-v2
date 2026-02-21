package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Subsystems.Lights;
@Config
public class AutoPresets {
    public static double xDif = 141.5;
    public static double xStartDif = 144;
    public static double calculateHeading(Lights.TeamColors team, double heading){
        if (team == Lights.TeamColors.BLUE) {
            return Math.toRadians(heading);
        }
        return Math.toRadians(180 - heading);
    }
    public static double calculateXStart(Lights.TeamColors team, double x){
        if (team == Lights.TeamColors.BLUE){
            return x;
        }
        return xStartDif - x;
    }
    public static double calculateX(Lights.TeamColors team, double x){
        if (team == Lights.TeamColors.BLUE){
            return x;
        }
        return xDif - x;
    }
}
