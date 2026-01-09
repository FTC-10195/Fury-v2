package org.firstinspires.ftc.teamcode.Subsystems.Sorting;

import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;

public class Ball {
    Sorter.Slot slot = Sorter.Slot.RIGHT;
    LimeLight.BallColors color = LimeLight.BallColors.NONE;
    public void setSlot(int id){
        slot = Sorter.idToSlot(id);
    }
    public void setSlot(Sorter.Slot slot){
        this.slot = slot;
    }
    public Sorter.Slot getSlot(){
        return slot;
    }
    public void setColor(LimeLight.BallColors color){
        this.color = color;
    }
    public LimeLight.BallColors getColor(){
        return color;
    }
    public static boolean full(Ball[] balls){
        int n = 0;
        for (int i = 0; i < balls.length; i++){
            if (balls[i].getColor() != LimeLight.BallColors.NONE){
                n++;
            }
        }
        return n == balls.length;
    }
    public static boolean empty(Ball[] balls){
        int n = 0;
        for (int i = 0; i < balls.length; i++){
            if (balls[i].getColor() == LimeLight.BallColors.NONE){
                n++;
            }
        }
        return n == balls.length;
    }
    public static Ball findBallOfColor(Ball[] balls, LimeLight.BallColors color){
        for (int i = 0; i < balls.length; i++){
            if (balls[i].getColor() == color){
                return balls[i];
            }
        }
        //No ball of target color? Find a ball.
        for (int i = 0; i < balls.length; i++){
            if (balls[i].getColor() != LimeLight.BallColors.NONE){
                return balls[i];
            }
        }
        //No balls at all????
        //Bruh, just return something
        return balls[0];
    }
    public static Ball findBallFromSlot(Ball[] balls, Sorter.Slot slot){
        for (int i = 0; i < balls.length; i++){
            if (balls[i].getSlot() == slot){
                return balls[i];
            }
        }
        //Default slot
        return balls[0];
    }
    public static boolean ThreeInARow(Ball[] balls){
        int n = 0;
        for (int i = 0; i < balls.length; i++){
            if (balls[i].getSlot() == Sorter.Slot.SHOOT ||balls[i].getSlot() ==  Sorter.Slot.INTAKE || balls[i].getSlot() == Sorter.Slot.CENTER){
                n++;
            }
        }
        return n == 3;
    }

}
