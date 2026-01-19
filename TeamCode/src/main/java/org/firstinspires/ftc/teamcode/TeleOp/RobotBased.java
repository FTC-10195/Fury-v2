package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp
public class RobotBased extends LinearOpMode {
    public enum States {
        RESTING,
        INTAKING,
        PREPARING_TO_FIRE,
        SHOOTING
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose holdPose = new Pose(0,0,0);
        boolean hold = false;
        States state = States.RESTING;
        waitForStart();
        FollowerHandler followerHandler = new FollowerHandler();
        followerHandler.initiate(hardwareMap);
        Follower follower = followerHandler.getFollower();
        follower.setStartingPose(new Pose(72,72));
        Flywheel flywheel = new Flywheel();
        flywheel.initiate(hardwareMap);
        Intake intake = new Intake();
        intake.initiate(hardwareMap);
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);
        Lights lights = new Lights();
        lights.initiate(hardwareMap);
        lights.setTeamColor(Lights.TeamColors.RED);
        Turret turret = new Turret();
        turret.initiate(hardwareMap);
        turret.setGoal(lights.getTeamColor());
        turret.setState(Turret.States.RESET);
        Gate gate = new Gate();
        gate.initiate(hardwareMap);

      //  LimeLight limeLight = new LimeLight();
     //   limeLight.initiate(hardwareMap);

        TelemetryPacket telemetryPacket = new TelemetryPacket(true);

        if (isStopRequested()) {
            lights.reset();
            followerHandler.reset();
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
            if (circle){
                followerHandler.forceRelocalize(lights.getTeamColor());
                gamepad1.rumble(1, 10, 100);
            }

            if (up){
                flywheel.add();
            }
            if (down){
                flywheel.sub();
            }

            if (options){
                lights.switchTeamColor();
            }
            if (LB) {
                state = States.RESTING;
            }
            if (RB){
                switch (turret.getState()){
                    case RESET:
                        turret.setState(Turret.States.AIM);
                      //  followerHandler.setPose(limeLight.relocalize(followerHandler.getFollower().getPose()));
                        break;
                    case AIM:
                        turret.setState(Turret.States.RESET);
                        break;
                }
            }
            switch (state) {
                case RESTING:
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.RESTING);
                    if (LT) {
                        state = States.INTAKING;}
                    if (RT) {
                        state = States.PREPARING_TO_FIRE;
                        flywheel.setState(Flywheel.States.SPINNING);}
                    break;
                case INTAKING:
                    intake.setState(Intake.States.ON);
                    flywheel.setState(Flywheel.States.RESTING);
                    lights.setMode(Lights.Mode.INTAKING);
                    if (LT ) {
                        state = States.RESTING;
                    }
                    break;
                case PREPARING_TO_FIRE:
                    intake.setState(Intake.States.OFF);
                    if (flywheel.isReady) {
                        gamepad1.rumble(1, 10, 100);
                        if (RT) {
                            gate.shoot();
                            state = States.SHOOTING;
                        }
                    }
                    break;
                case SHOOTING:
                    intake.setState(Intake.States.ON);
                    if (gate.doneShooting()){
                        state = States.RESTING;
                    }
                    break;
            }

            //Overrides
            if (gamepad1.cross) {
                intake.setState(Intake.States.OUTTAKE);
            }
            if (square) {
                hold = !hold;
                if (hold){
                    holdPose = followerHandler.getFollower().getPose();
                    followerHandler.setBrakeMode();

                    followerHandler.getFollower().updateConstants();
                    followerHandler.getFollower().holdPoint(holdPose);
                }else{
                    followerHandler.getFollower().breakFollowing();
                }
            }

        //    lights.setMotif(limeLight.getMotif());
            turret.setGoal(lights.getTeamColor());
            turret.setPose(followerHandler.getFollower().getPose());
            flywheel.calculateZone(followerHandler.getFollower().getPose(),lights.getTeamColor());


       //     limeLight.update(telemetry);


            flywheel.update();
            turret.update();
            intake.update();
            gate.update();
            followerHandler.update();
            if (!hold) {
                drivetrain.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }



            telemetry.addData("State", state);
            telemetry.addData("Hold", hold);
            telemetry.addData("HoldPose",holdPose);

            telemetry.addData("X", followerHandler.getFollower().getPose().getX());
            telemetry.addData("Y", followerHandler.getFollower().getPose().getY());
            telemetry.addData("Heading", followerHandler.getFollower().getPose().getHeading());

            lights.update(telemetry);

            flywheel.status(telemetry);
            turret.status(telemetry);
            telemetry.update();

      //      limeLight.ftcDashUpdate(telemetryPacket);
            flywheel.updateTelemetryPacket(telemetryPacket);
            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);

        }
        lights.reset();
    }
}
