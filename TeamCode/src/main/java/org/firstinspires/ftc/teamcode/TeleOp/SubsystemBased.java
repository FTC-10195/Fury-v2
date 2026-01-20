package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;

@TeleOp
public class SubsystemBased extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        FollowerHandler followerHandler = new FollowerHandler();
        followerHandler.initiate(hardwareMap);
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);
        Lights lights = new Lights();
        lights.initiate(hardwareMap);
        Intake intake = new Intake();
        intake.initiate(hardwareMap);
        Flywheel flywheel = new Flywheel();
        flywheel.initiate(hardwareMap);
        Gate gate = new Gate();
        gate.initiate(hardwareMap);




        TelemetryPacket telemetryPacket = new TelemetryPacket(true);

        if (isStopRequested()) {
            lights.reset();
            followerHandler.reset();
            return;
        }

        Gamepad previousGamepad1 = new Gamepad();
        while (opModeIsActive()) {

            boolean triangle = gamepad1.triangle && !previousGamepad1.triangle;
            boolean X = gamepad1.cross && !previousGamepad1.cross;
            boolean circle = gamepad1.circle && !previousGamepad1.circle;
            boolean square = gamepad1.square && !previousGamepad1.square;
            boolean RB = gamepad1.right_bumper && !previousGamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper && !previousGamepad1.left_bumper;
            boolean LT = gamepad1.left_trigger >= .1 && previousGamepad1.left_trigger < .1;
            boolean RT = gamepad1.right_trigger >= .1 && previousGamepad1.right_trigger < .1;
            boolean options = gamepad1.options && !previousGamepad1.options;

            previousGamepad1.copy(gamepad1);

            if (triangle){
                lights.switchTeamColor();
            }

            if (square) {
                followerHandler.flipLock();
            }
            if (RB){
                followerHandler.forceRelocalize(lights.getTeamColor());
            }
            if (LT){
                intake.flipState();
            }
            if (RB){
                flywheel.flipState();
            }
            if (LB){
                intake.setState(Intake.States.OFF);
                flywheel.setState(Flywheel.States.PASSIVE);
            }
            if (RT){
                gate.shoot();
            }
            if (X){
                if (intake.getState() == Intake.States.EJECT){
                    intake.setState(Intake.States.OFF);
                }else {
                    intake.setState(Intake.States.EJECT);
                }
            }

            followerHandler.update();
            flywheel.calculateZone(followerHandler.getFollower().getPose(),lights.getTeamColor());
            lights.update(telemetry);
            intake.update();
            flywheel.update();
            gate.update();

            if (!followerHandler.isLocked()) {
                drivetrain.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            drivetrain.status(telemetry);
            followerHandler.status(telemetry);
            telemetry.update();

            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);

        }
    }
}
