package org.firstinspires.ftc.teamcode.Subsystems.Sorting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
@Config
public class Sorter {
    public enum Slot {
        RIGHT,
        LEFT,
        CENTER,
        SHOOT,
        INTAKE
    }
    public enum State{
        RESTING,
        INTAKING,
        CHAMBER,
        SHOOTING
    }
    public enum Mode{
        UNSORTED,
        SORTED
    }

    BallDetector ballDetector = new BallDetector();
    Slots slots = new Slots();
    Transfer transfer;
    Intake intake;
    private Mode mode = Mode.UNSORTED;
    private State state = State.RESTING;

    /*
    0 -> right
    1 -> left
    2 -> intake
     */
    Ball[] balls = {new Ball(), new Ball(), new Ball()};
    LimeLight.BallColors[] motif = {LimeLight.BallColors.P, LimeLight.BallColors.P, LimeLight.BallColors.G};



    public void initiate(HardwareMap hardwareMap, Intake intake, Transfer transfer){
        slots.initiate(hardwareMap,intake,transfer);
        this.intake = intake;
        this.transfer = transfer;
    }
    public Slots getSlot(){
        return slots;
    }
    public BallDetector getBallDetector(){
        return ballDetector;
    }


    public static Slot idToSlot(int id){
        switch (id){
            case 0:
                return Sorter.Slot.RIGHT;
            case 1:
                return Sorter.Slot.LEFT;
            default:
                return Sorter.Slot.INTAKE;
        }
    }


    public Mode getMode(){
        return mode;
    }
    public void setMode(Mode mode){
        this.mode = mode;
    }
    public State getState(){
        return state;
    }
    public void setState(State state){
        this.state = state;
    }
    public void setMotif(LimeLight.BallColors[] motif){
        this.motif = motif;
    }

    public int ballToSlot(Slot ballSlot, Slot targetSlot){
        int n = 0;
        if (ballSlot == Slot.RIGHT && targetSlot == Slot.CENTER){
            n = slots.rightToCenterAction();
        }else if (ballSlot == Slot.RIGHT && targetSlot == Slot.SHOOT){
            n = slots.rightToShootAction();
        }else if (ballSlot == Slot.LEFT && targetSlot == Slot.CENTER){
            n = slots.leftToCenterAction();
        }else if (ballSlot == Slot.LEFT && targetSlot == Slot.SHOOT){
            n = slots.leftToShootAction();
        }else if (ballSlot == Slot.INTAKE && targetSlot == Slot.CENTER){
            n = slots.intakeToCenterAction();
        }else {
            n = slots.intakeToShootAction();
        }
        if (n == 1){
            //Update ball slot when its done moving
            Ball.findBallFromSlot(balls,ballSlot).setSlot(targetSlot);
        }
        return n;

    }

    int currentBall = 0;
    int sequence = 0;
    public void reset(){
        //Reset the ball count, main issue here is if somehow the ball counting gets off, idk what to do about that since no way to check balls that are already inside slots
        currentBall = 0;
        sequence = 0;
        slots.toDefault();
        state = State.RESTING;
    }

    public void intakeSequence(){
        if (Ball.full(balls) || currentBall == 3){
            //The last ball is detected, stop intaking and transfering
            intake.setState(Intake.States.OFF);
            transfer.setState(Transfer.States.OFF);
            reset();
            return;
        }

        //No balls yet? bring the right slot over
        if (currentBall == 0 && sequence == 0){
            slots.rightToCenter();
        }

        switch (sequence){
            case 0:
                //No ball? No go
                if (!ballDetector.isBall()){
                    return;
                }
                balls[currentBall].setSlot(currentBall);
                balls[currentBall].setColor(ballDetector.getBallColor());
                sequence++;
                break;
            case 1:
                sequence += slots.intakeToCenterAction();
                break;
            case 2:
                if (currentBall == 0) {
                    sequence += slots.leftToCenterAction();
                }else if (currentBall == 1){
                    sequence += slots.toDefaultAction();
                }else{
                    sequence++;
                }
                if (sequence != 2){
                    currentBall++;
                    sequence = 0;
                    return;
                }
                break;
        }
    }
    public void chamberSequence(){
        // currentBall == 2 because 0, 1, 2 is order. 3rd ball doesn't need any action
        if (Ball.empty(balls) || currentBall == 2){
            //The last ball is detected, stop intaking and transfering
            transfer.setState(Transfer.States.OFF);
            reset();
            return;
        }

        //Okay we have balls, now lets shoot them
        //Find ball of target color
        Ball ball = Ball.findBallOfColor(balls,motif[currentBall]);
        //Move to shoot if first ball
        switch (currentBall) {
            case 0:
                currentBall += ballToSlot(ball.getSlot(), Slot.SHOOT);
                break;
            case 1:
                currentBall += ballToSlot(ball.getSlot(),Slot.CENTER);
                break;
        }
        //At this point nothing to do. Never move last ball, it cant go anywhere


    }
    public void shootSequence(){
        if (Ball.ThreeInARow(balls)){
            //Spin transfer, intake and flywheel -> Done
            return;
        }
        //First 2 shot? -> Move last one and shoot
        ballToSlot(balls[2].getSlot(),Slot.CENTER);
    }

    public void update(){
        slots.update();
        if (mode == Mode.UNSORTED){
            return;
        }
        switch (state){
            case RESTING:
                slots.toDefault();
                break;
            case CHAMBER:
                chamberSequence();
                break;
            case INTAKING:
                intakeSequence();
                break;
        }
    }

}
