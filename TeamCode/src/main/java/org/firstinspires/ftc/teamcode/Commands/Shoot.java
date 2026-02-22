package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Subsystems.EverythingThatNeedsLocalization.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Config
public class Shoot {
    boolean initial = true;
    boolean done = false;
    public boolean isDone(){
        return done;
    }
    public int run(Intake intake, Gate gate, Flywheel flywheel, boolean flywheelStaySpinning){
        if (initial) {
            gate.shoot();
            intake.setState(Intake.States.OFF);
            initial = false;
            done = false;
            return 0;
        }
        if (gate.getState() == Gate.State.OPENNING){
            return 0;
        }
        if (gate.getState() == Gate.State.OPEN){
            flywheel.shooting = true;
        intake.setState(Intake.States.SHOOTING);
            return 0;
        }
        if (gate.getState() == Gate.State.CLOSING || gate.getState() == Gate.State.CLOSE){
            intake.setState(Intake.States.OFF);
            initial = true;
            done = true;
            flywheel.setState(Flywheel.States.PASSIVE);
            if (!flywheelStaySpinning) {
                flywheel.shooting = false;
            }
            return 1;
        }
        return 0;
    }
    public int run(Intake intake, Gate gate, Flywheel flywheel) {
        return run(intake,gate,flywheel,false);
    }


}
