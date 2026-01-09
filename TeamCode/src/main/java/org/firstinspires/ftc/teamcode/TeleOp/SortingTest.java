package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.Sorting.Sorter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp
public class SortingTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Intake intake = new Intake();
        intake.initiate(hardwareMap);
        Transfer transfer = new Transfer();
        transfer.initiate(hardwareMap);
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);
        Lights lights = new Lights();
        lights.initiate(hardwareMap);
        lights.setTeamColor(Lights.TeamColors.RED);
        Sorter sorter = new Sorter();
        sorter.initiate(hardwareMap,intake,transfer);



        TelemetryPacket telemetryPacket = new TelemetryPacket(true);

        if (isStopRequested()) {
            lights.reset();
            return;
        }

        Gamepad previousGamepad1 = new Gamepad();
        while (opModeIsActive()) {

            boolean LB = gamepad1.left_bumper && !previousGamepad1.left_bumper;
            boolean RB = gamepad1.right_bumper && !previousGamepad1.right_bumper;
            boolean X = gamepad1.cross && !previousGamepad1.cross;
            boolean square = gamepad1.square && !previousGamepad1.square;
            boolean triangle = gamepad1.triangle && !previousGamepad1.triangle;
            boolean LT = gamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger <= 0.1;
            boolean RT = gamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger <= 0.1;
            boolean circle = gamepad1.circle && !previousGamepad1.circle;
            boolean options = gamepad1.options && !previousGamepad1.options;
            boolean up = gamepad1.dpad_up && !previousGamepad1.dpad_up;
            boolean down = gamepad1.dpad_down && !previousGamepad1.dpad_down;
            previousGamepad1.copy(gamepad1);

            if (LT){
                sorter.getSlot().leftToCenter();
            }
            if (RT){
                sorter.getSlot().rightToCenter();
            }
            if (RB){
                sorter.getSlot().centerToShootAction();
            }
            if (LB){
                sorter.getSlot().toDefault();
            }
            lights.setMode(Lights.Mode.INTAKING);
            lights.setBall(sorter.getBallDetector().getBallColor());

            sorter.update();
            intake.update();
            transfer.update();
            lights.update(telemetry);
            telemetry.update();


        }
        lights.reset();
    }
}
