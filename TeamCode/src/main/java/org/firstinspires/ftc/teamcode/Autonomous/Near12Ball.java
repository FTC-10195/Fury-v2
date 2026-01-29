package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.FollowerHandler;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Autonomous
public class Near12Ball extends LinearOpMode {
    private static final Logger log = LoggerFactory.getLogger(Near12Ball.class);
    private Follower follower;
    private Flywheel flywheel = new Flywheel();
    private Intake intake = new Intake();
    private Lights lights = new Lights();
    private Gate gate = new Gate();
    private Turret turret = new Turret();
    FollowerHandler followerHandler = new FollowerHandler();
    Command command;
    private int path = 0;

    private double calculateHeading(double heading) {
        if (lights.getTeamColor() == Lights.TeamColors.BLUE) {
            return Math.toRadians(heading);
        }
        return Math.toRadians(180 - heading);
    }
    private double calculateX(double x){
        if (lights.getTeamColor() == Lights.TeamColors.BLUE){
            return x;
        }
        return 144 - x;
    }

    PathChain shootPrescore,
            intakeFirst, openGate, shoot2, intakeSecond,shoot3, intakeThird, shoot4, leave;

    public void buildPaths() {

        final Pose startPose = new Pose(calculateX(33.34341906202723), 132.393343419062, calculateHeading(180)); // Start Pose of our robot.
        final Pose shootPose = new Pose(calculateX(53.20830181543116), 85.8124054462935, calculateHeading(180));

        final Pose intakePose1 = new Pose(calculateX(12.098335854765496), 58.38880484114973, calculateHeading(180));
        final Pose intakeControl1 = new Pose(calculateX(54.778129727685325),53.09909228441754);

        final Pose gateOpenPose = new Pose(calculateX(14.576399394856292),68.71406959152799,calculateHeading(180));
        final Pose gateOpenControl = new Pose(calculateX(35.56127080181543),59.97655068078665,calculateHeading(180));

        final Pose shootPose2 = new Pose(calculateX(55.63842662632375), 79.80332829046898, calculateHeading(180));
        final Pose shootControl2 = new Pose(calculateX(41.81996974281392),62.75869894099847);
        final Pose intakePose2 = new Pose(calculateX(21.583963691376697), 85.49924357034797, calculateHeading(180));

        final Pose intakePose3 = new Pose(calculateX(18.461422087745841), 38.236006051437215, calculateHeading(180));
        final Pose intakeControl3 = new Pose(calculateX(64.728441754916794), 25.568078668683814);
        final Pose leavePose = new Pose(calculateX(32.161875945537055),66.82904689863841, calculateHeading(180));





        followerHandler.setStartingPose(startPose);
        followerHandler.save();

        shootPrescore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                shootPose
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                intakeControl1,
                                intakePose1
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        openGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                intakePose1,
                                gateOpenControl,
                                gateOpenPose
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                gateOpenPose,
                                shootControl2,
                                shootPose2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose2,
                                intakePose2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose2,
                                shootPose2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose2,
                                intakeControl3,
                                intakePose3
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose3,
                                shootPose2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose2,
                                leavePose
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad previousGamepad1 = new Gamepad();
        followerHandler.initiate(hardwareMap);
        follower = followerHandler.getFollower();

        flywheel.initiate(hardwareMap);
        intake.initiate(hardwareMap);
        lights.initiate(hardwareMap);
        lights.setTeamColor(Lights.TeamColors.BLUE);
        gate.initiate(hardwareMap);
        turret.initiate(hardwareMap);
        turret.setState(Turret.States.AIM);

        command = new Command(flywheel, gate, intake, followerHandler);

        //Chamber
        boolean initial = true;
        while (!opModeIsActive() && !isStopRequested()) {
            if (initial){
                turret.setState(Turret.States.RESET);
                initial = false;
            }
            boolean options = gamepad1.options && !previousGamepad1.options;
            boolean rb = gamepad1.right_bumper && !previousGamepad1.right_bumper;
            previousGamepad1.copy(gamepad1);

            if (options) {
                lights.switchTeamColor();
                buildPaths();
            }
            if (rb){
                buildPaths();
            }

            lights.update(telemetry);
            telemetry.update();
            turret.update();

        }

        waitForStart();
        buildPaths();

        //   LimeLight limeLight = new LimeLight();
        //  limeLight.initiate(hardwareMap);
        if (isStopRequested()) {
            lights.save();
            return;
        }
        while (opModeIsActive()) {
            telemetry.addData("Auto path", path);

            followerHandler.update();
            followerHandler.save();
            flywheel.auto = true;
            turret.setFollowerHandler(followerHandler);
            turret.setGoal(lights.getTeamColor());
            turret.update();

            flywheel.update();
            intake.update();
            lights.update(telemetry);
            lights.save();
            gate.update();


            flywheel.status(telemetry);
            gate.status(telemetry);


            telemetry.update();

            switch (path) {
                case 0:
                    turret.setState(Turret.States.MANUAL);
                    turret.calculateOverrideAngle(lights.getTeamColor(),-45);
                    path += command.runFollow(shootPrescore, 1500);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 1:
                    path += command.runShoot();
                    break;
                case 2:
                    path += command.runFollow(intakeFirst,2000,.8);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 3:
                    path += command.runFollow(openGate,1500);
                    intake.setState(Intake.States.OFF);
                    break;
                case 4:
                    path += command.runFollow(shoot2,1200);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 5:
                    path += command.runShoot();
                    break;
                case 6:
                    path += command.runFollow(intakeSecond,1500);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 7:
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    path += command.runFollow(shoot3,1200);
                    break;
                case 8:
                    path += command.runShoot();
                    break;
                case 9:
                    intake.setState(Intake.States.INTAKE);
                    path += command.runFollow(intakeThird,3000);
                    break;
                case 10:
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    path += command.runFollow(shoot4,2000);
                    break;
                case 11:
                    path += command.runShoot();
                    break;
                case 12:
                    path += command.runFollow(leave,1000);
                    break;

            }

        }
    }
}
