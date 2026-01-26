package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Config
public class Shoot {
    boolean initial = true;
    boolean done = false;
    public boolean isDone(){
        return done;
    }
    public int run(Intake intake, Gate gate, Flywheel flywheel){
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
            intake.setState(Intake.States.INTAKE);
            return 0;
        }
        if (gate.getState() == Gate.State.CLOSING || gate.getState() == Gate.State.CLOSE){
            intake.setState(Intake.States.OFF);
            initial = true;
            done = true;
            flywheel.setState(Flywheel.States.PASSIVE);
            return 1;
        }
        return 0;
    }

}
