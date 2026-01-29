package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
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

        LimeLight limeLight = new LimeLight();
        limeLight.initiate(hardwareMap);

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
            boolean square = gamepad1.square && !previousGamepad1.square;
            boolean triangle = gamepad1.triangle && !previousGamepad1.triangle;
            boolean LT = gamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger <= 0.1;
            boolean RT = gamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger <= 0.1;
            boolean circle = gamepad1.circle && !previousGamepad1.circle;
            boolean options = gamepad1.options && !previousGamepad1.options;
            boolean share = gamepad1.share && !previousGamepad1.share;
            boolean up = gamepad1.dpad_up && !previousGamepad1.dpad_up;
            boolean down = gamepad1.dpad_down && !previousGamepad1.dpad_down;
            boolean left = gamepad1.dpad_left && !previousGamepad1.dpad_left;

            previousGamepad1.copy(gamepad1);

            if (circle){
               // drivetrain.flipMode();
                gamepad1.rumble(1, 10, 100);
                followerHandler.getFollower().setPose(limeLight.relocalize(followerHandler.getFollower().getPose()));
            }
            if (triangle){
                flywheel.on = !flywheel.on;
            }
            if (share){
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
                turret.on = !turret.on;
            }
            switch (state) {
                case RESTING:
                    turret.setState(Turret.States.RESET);
                    flywheel.shooting = false;
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.PASSIVE);
                    if (LT) {
                        state = States.INTAKING;
                    }
                    if (RT) {
                        state = States.PREPARING_TO_FIRE;
                    }
                    break;
                case INTAKING:
                    turret.setState(Turret.States.RESET);
                    flywheel.shooting = false;
                    intake.setState(Intake.States.INTAKE);
                    flywheel.setState(Flywheel.States.PASSIVE);
                   // lights.setMode(Lights.Mode.INTAKING);
                    if (LT) {
                        state = States.RESTING;
                    }
                    if (RT) {
                        state = States.PREPARING_TO_FIRE;
                    }
                    break;
                case PREPARING_TO_FIRE:
                    flywheel.shooting = false;
                    turret.setState(Turret.States.AIM);
                    flywheel.setState(Flywheel.States.SPINNING);
                    if (LT) {
                        state = States.INTAKING;
                    }
                    intake.setState(Intake.States.OFF);
                    if (flywheel.isReady) {
                        if (flywheel.withinTolerance()) {
                            gamepad1.rumble(1, 10, 100);
                        }
                        if (RT) {
                            gate.shoot();
                            state = States.SHOOTING;
                        }
                    }
                    break;
                case SHOOTING:
                    turret.setState(Turret.States.AIM);
                    flywheel.setState(Flywheel.States.SPINNING);
                    intake.setState(Intake.States.SHOOTING);
                    flywheel.shooting = true;
                    if (gate.doneShooting()){
                        state = States.RESTING;
                    }
                    break;
            }

            //Overrides
            if (gamepad1.cross) {
                intake.setState(Intake.States.EJECT);
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
            turret.setFollowerHandler(followerHandler);
            flywheel.calculateZone(followerHandler.getFollower().getPose(),lights.getTeamColor());
            intake.setZone(flywheel.getZone());
            drivetrain.setYaw(followerHandler.getFollower().getHeading());


            limeLight.update(telemetry);


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
            followerHandler.status(telemetry);
            intake.update();
            telemetry.update();

      //      limeLight.ftcDashUpdate(telemetryPacket);
            flywheel.updateTelemetryPacket(telemetryPacket);
            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);

        }
        lights.reset();
    }
}
