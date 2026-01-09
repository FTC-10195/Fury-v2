package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Timer;

public class Wait {
    Timer timer = new Timer();
    int sequence = 0;
    public int run(long time){
        switch (sequence){
            case 0:
                timer.setWait(time);
            break;
            case 1:
                if (timer.doneWaiting()){
                    sequence = 0;
                    return 1;
                }
                break;
        }
        return 0;
    }

}
