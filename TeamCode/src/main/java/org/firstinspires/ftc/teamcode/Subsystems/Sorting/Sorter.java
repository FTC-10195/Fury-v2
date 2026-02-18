package org.firstinspires.ftc.teamcode.Subsystems.Sorting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
@Config
public class Sorter {
    public static double rightServoRightPos = 0.3;
    public static double rightServoCenterPos = 0.5;
    public static double leftServoLeftPos = 0.8;
    public static double leftServoCenterPos = 0.5;
    Sorter.States currentState = States.OFF;
    Servo rightServo;
    Servo leftServo;
    BallDetector ballDetector = new BallDetector();
    Slot rightSlot = new Slot();
    Slot leftSlot = new Slot();
    Slot[] slots = {rightSlot, leftSlot};
    int t = 0;
    LimeLight.BallColors[] motif = {LimeLight.BallColors.P, LimeLight.BallColors.P, LimeLight.BallColors.G};

    public void setState(States state) {
        currentState = state;
        if (state == States.INTAKING) {
            rightToCenter();
        }
    }

    public void initiate(HardwareMap hardwareMap) {
        rightServo = hardwareMap.servo.get("rc");
        leftServo = hardwareMap.servo.get("lc");
        leftSlot.currentPosition = Slot.Position.LEFT;
        leftSlot.defaultPosition = Slot.Position.LEFT;
        rightSlot.currentPosition = Slot.Position.RIGHT;
        rightSlot.defaultPosition = Slot.Position.RIGHT;
        ballDetector.initiate(hardwareMap);

    }

    public void toRight() {
        rightServo.setPosition(rightServoRightPos);
        rightSlot.currentPosition = Slot.Position.RIGHT;
    }

    public void toLeft() {
        leftServo.setPosition(leftServoLeftPos);
        leftSlot.currentPosition = Slot.Position.LEFT;
    }

    public void rightToCenter() {
        toLeft();
        rightServo.setPosition(rightServoCenterPos);
        rightSlot.currentPosition = Slot.Position.CENTER;
    }

    public void leftToCenter() {
        toRight();
        leftServo.setPosition(leftServoCenterPos);
        leftSlot.currentPosition = Slot.Position.CENTER;
    }

    public void setMotif(LimeLight.BallColors[] motif) {
        this.motif = motif;
    }
    int numberOfBallsIntaked = 0;
    public void intake() {
        if (ballDetector.isBall()) {
            return;
        }
        if (numberOfBallsIntaked >= 3){
            return;
        }
        switch (numberOfBallsIntaked) {
            case 0:
                numberOfBallsIntaked++;
                if (motif[t] == ballDetector.getBallColor()) {
                    //Let the ball roll to shoot zone
                    t++;
                    return;
                }
                //Ball goes into right slot
                rightSlot.color = ballDetector.getBallColor();
                leftToCenter();
                return;
            case 1:
                numberOfBallsIntaked++;
                //t = 0 means left in center and right in right
                if (t == 0){
                    leftSlot.color = ballDetector.getBallColor();
                    if (motif[t] == ballDetector.getBallColor()){
                        //Center slot -> left
                        t++;
                        return;
                    }
                    toLeft();
                }else if (t == 1){
                    rightSlot.color = ballDetector.getBallColor();
                    leftToCenter();
                    t++;
                }
                break;
            case 2:
                numberOfBallsIntaked++;
                Slot.findSlotWithColor(slots, LimeLight.BallColors.NONE).color = ballDetector.getBallColor();

                Slot slot = Slot.findSlotWithColor(slots,motif[1]);
                if (slot == null){
                    return;
                }
                if (slot.defaultPosition == Slot.Position.RIGHT){
                    rightToCenter();
                }else{
                    leftToCenter();
                }
                break;
        }
    }

    public void shoot() {
        //Shoot
        //Wait X seconds
        Slot targetSlot = Slot.findSlotNotAtCenter(slots);
        if (targetSlot.currentPosition == Slot.Position.RIGHT){
            rightToCenter();
        }else if (targetSlot.currentPosition == Slot.Position.LEFT){
            leftToCenter();
        }

    }

    public void update() {
        switch (currentState) {
            case RESTING:
            case OFF:
                toRight();
                toLeft();
                numberOfBallsIntaked = 0;
                t = 0;
                break;
            case INTAKING:
                intake();
                break;
            case SHOOTING:
                shoot();
        }

    }
    public void status(Telemetry telemetry){
        telemetry.addLine("SORTER ------");
        telemetry.addData("Sorter State",currentState);
        telemetry.addData("NumberOfBallsIntaked",numberOfBallsIntaked);
        telemetry.addData("t",t);
    }

    public enum States {
        SHOOTING,
        INTAKING,
        OFF,
        RESTING;
    }

}
