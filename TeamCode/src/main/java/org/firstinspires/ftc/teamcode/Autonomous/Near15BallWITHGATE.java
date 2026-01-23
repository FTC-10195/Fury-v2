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
public class Near15BallWITHGATE extends LinearOpMode {
    private static final Logger log = LoggerFactory.getLogger(Near15BallWITHGATE.class);
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
            intakeFirst, shoot2, intakeSecond,shoot3, intakeThird, shoot4, intakeFourth, shoot5, leave;

    public void buildPaths() {

        final Pose startPose = new Pose(calculateX(33.34341906202723), 132.393343419062, calculateHeading(180)); // Start Pose of our robot.
        final Pose shootPose = new Pose(calculateX(53.20830181543116), 85.8124054462935, calculateHeading(180));
        final Pose intakePose1 = new Pose(calculateX(14.583963691376697), 64.70650529500753, calculateHeading(180));
        final Pose intakeControl1 = new Pose(calculateX(44.778129727685325),53.09909228441754);
        final Pose shootPose2 = new Pose(calculateX(55.63842662632375), 79.80332829046898, calculateHeading(180));
        final Pose shootControl2 = new Pose(calculateX(41.81996974281392),62.75869894099847);
        final Pose intakePose2 = new Pose(calculateX(17.56278366111951), 75.84266263237518, calculateHeading(180));
        final Pose intakeControl2 = new Pose(calculateX(54.414523449319205), 93.89409984871406);

        final Pose intakePose3 = new Pose(calculateX(14), 38, calculateHeading(180));
        final Pose intakeControl3 = new Pose(calculateX(74.728441754916794), 20.568078668683814);
        final Pose shootPose3 = new Pose(calculateX(53.5249621785174),79.36308623298032, calculateHeading(180));
        final Pose intakePose4 = new Pose(calculateX(8.084720121028745),9.060514372163404,calculateHeading(180));
        final Pose intakeControl4 = new Pose(calculateX(46.160363086232984),32.1142208774584);

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
        shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                intakePose1,
                                shootControl2,
                                shootPose2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose2,
                                intakeControl2,
                                intakePose2
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose2,
                                shootPose3
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose3,
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
         intakeFourth = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose2,
                                intakeControl4,
                                intakePose4
                        )
                )
                .setGlobalConstantHeadingInterpolation(calculateHeading(180))
                .build();
        shoot5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose4,
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
            turret.setPose(follower.getPose());
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
                    turret.setState(Turret.States.AIM);
                    path += command.runFollow(shootPrescore, 1000);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 1:
                    path += command.runShoot();
                    break;
                case 2:
                    path += command.runFollow(intakeFirst,2000);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 3:
                    intake.setState(Intake.States.OFF);
                    path += command.runFollow(shoot2,900);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 4:
                    path += command.runShoot();
                    break;
                case 5:
                    path += command.runFollow(intakeSecond,2000);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 6:
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    path += command.runFollow(shoot3,1000);
                    break;
                case 7:
                    path += command.runShoot();
                    break;
                case 8:
                    path += command.runFollow(intakeThird,2500);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 9:
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    path += command.runFollow(shoot4,1200);
                    break;
                case 10:
                    path += command.runShoot();
                    break;
                case 11:
                    path += command.runFollow(intakeFourth,2500);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 12:
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    path += command.runFollow(shoot5,2000);
                    break;
                case 13:
                    path += command.runShoot();
                    break;
                case 14:
                    path += command.runFollow(leave,1500);
                    intake.setState(Intake.States.OFF);
                    break;
                case 15:
                    break;


            }

        }
    }
}
