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

@Autonomous
public class GateIntake extends LinearOpMode {
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
            intakeFirst, shoot2, intakeSecond,shoot3, intakeThird, shoot4, leave;

    public void buildPaths() {

        final Pose startPose = new Pose(calculateX(33.34341906202723), 132.393343419062, calculateHeading(180)); // Start Pose of our robot.
        final Pose shootPose = new Pose(calculateX(53.20830181543116), 85.8124054462935, calculateHeading(180));

        final Pose intakePose1 = new Pose(calculateX(14.662632375189096), 57.08169440242053, calculateHeading(180));
        final Pose intakeControl1 = new Pose(calculateX(54.778129727685325),50.70272314674733);


        final Pose shootPose2 = new Pose(calculateX(55.63842662632375), 79.80332829046898, calculateHeading(180));
        final Pose shootControl2 = new Pose(calculateX(54.56429652042359),58.996217851739765);

        final Pose intakePose2 = new Pose(calculateX(14.042965204236004),59.076096822995456,calculateHeading(167));



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
                        new BezierLine(
                                shootPose2,
                                intakePose2
                        )
                )
                .setLinearHeadingInterpolation(calculateHeading(180),intakePose2.getHeading())
                .build();
        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakePose2,
                                shootPose2
                        )
                )
                .setLinearHeadingInterpolation(intakePose2.getHeading(),calculateHeading(180))
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
            boolean triangle = gamepad1.triangle && !previousGamepad1.triangle;
            boolean rb = gamepad1.right_bumper && !previousGamepad1.right_bumper;
            previousGamepad1.copy(gamepad1);

            if (options || triangle) {
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
                    path += command.runFollow(shootPrescore, 1700);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 1:
                    path += command.runShoot();
                    break;
                case 2:
                    path += command.runFollow(intakeFirst,1900);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 3:
                    path += command.runFollow(shoot2,1700);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 4:
                    path += command.runShoot();
                    break;
                case 5:
                    path += command.runFollow(intakeSecond,3200,.8);
                    intake.setState(Intake.States.INTAKE);
                    break;
                case 6:
                    path += command.runFollow(shoot3,1700);
                    intake.setState(Intake.States.OFF);
                    flywheel.setState(Flywheel.States.SPINNING);
                    break;
                case 7:
                    path += command.runShoot();
                    break;


            }

        }
    }
}
