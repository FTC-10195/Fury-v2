package org.firstinspires.ftc.teamcode.Subsystems.Sorting;

import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;

public class Slot {
    public enum Position{
        RIGHT,
        CENTER,
        LEFT
    }
    public LimeLight.BallColors color = LimeLight.BallColors.NONE;
    public Position currentPosition = Position.RIGHT;
    public Position defaultPosition = Position.RIGHT;

    public static boolean empty(Slot[] slots){
        for (int i = 0; i < slots.length; i++){
            if (slots[i].color != LimeLight.BallColors.NONE){
                return false;
            }
        }
        return true;
    }
    public static Slot findSlotNotAtCenter(Slot[] slots){
        for (int i = 0; i < slots.length; i++){
            if (slots[i].currentPosition != Position.CENTER){
                return slots[i];
            }
        }
        return slots[0];
    }

    public static Slot findSlotWithBall(Slot[] slots){
        int id = 0;
        for (int i = 0; i < slots.length; i++){
            if (slots[i].color != LimeLight.BallColors.NONE){
                id = i;
            }
        }
        return slots[id];
    }
    public static Slot findSlotWithColor(Slot[] slots,LimeLight.BallColors color){
        Slot slot = null;
        for (int i = 0; i < slots.length; i++){
            if (slots[i].color == color){
                slot = slots[i];
            }
        }
        if (slot == null){
            slot = findSlotWithBall(slots);
        }
        return slot;
    }


}
