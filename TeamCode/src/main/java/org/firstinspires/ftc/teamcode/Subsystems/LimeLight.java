package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;

@Config
public class LimeLight {
    public enum BallColors {
        P,
        G,
        NONE
    }
    public enum Mode{
        RELOCALIZE,
        RESTING
    }
    Mode mode = Mode.RESTING;
    public Mode getMode(){
        return mode;
    }

    public static Pose redGoalPos = new Pose(128.5446293494705, 132.61119515885022, Math.toRadians(40));
    public static Pose blueGoalPos = new Pose(16, 132.61119515885022, Math.toRadians(140));
    public static double distanceThreshold = 4;
    public static double headingThresholdDegrees = 5;
    public static int numberOfLocalizationAttempts = 8;
    public static long relocalizationOverrideTime = 500;
    public static long relocalizationCooldownTime = 10000;
    Timer relocalizeOverideTimer = new Timer();
    Timer relocalizeCooldownTimer = new Timer();

    static BallColors[] motif = {BallColors.P, BallColors.P, BallColors.G};

    public BallColors[] getMotif() {
        return motif;
    }

    public static final int blueId = 20;
    public static final int redId = 24;
    public static int GPPId = 21;
    public static int PGPId = 22;
    public static int PPGId = 23;
    public static double velocityCountsAsMovingThreshold = 5;

    public static Pose getPoseFromId(int i) {
        Pose pose = new Pose(0, 0, 0);
        switch (i) {
            case blueId:
                pose = blueGoalPos;
                break;
            case redId:
                pose = redGoalPos;
                break;
        }
        return pose;
    }

    /*public static double camForward = 0;
    public static double camVertical = 0;
    public static double camHorizontal = 0;
    public static double yaw = 0;
    public static double pitch = 0;
    public static double roll = 0;

     */
    Pose pedroPose = new Pose(0, 0, 0);
    Pose relocalizePose = pedroPose;

    public boolean canRelocalize = false;
    public boolean localized = false;
    int localizeSequence = 0;
    int relocalizingId = 0;
    public boolean automaticRelocalization = true;
    Limelight3A limelight;

    ArrayList<Pose> relocalizePositions = new ArrayList<>();

    public static String ballToString(BallColors color) {
        switch (color) {
            case P:
                return "P";
            case G:
                return "G";
            case NONE:
                return "NONE";
        }
        return "";
    }

    public static String motifToString(BallColors[] motif) {
        return ballToString(motif[0]) + ballToString(motif[1]) + ballToString(motif[2]);
    }

    public void initiate(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public boolean motifId(int id) {
        return id == PPGId || id == PGPId || id == GPPId;
    }

    public void setMotif(int id) {
        if (id == GPPId) {
            motif = new BallColors[]{BallColors.G, BallColors.P, BallColors.P};
        } else if (id == PGPId) {
            motif = new BallColors[]{BallColors.P, BallColors.G, BallColors.P};
        } else if (id == PPGId) {
            motif = new BallColors[]{BallColors.P, BallColors.P, BallColors.G};
        }
    }

    public static Pose translateLimelightPoseToPedro(Position position, double yaw) {
        //Step 1 flip x and y
        double savedX = position.x;
        position.x = position.y;
        position.y = savedX;

        //Step 2 add 72 to x
        position.x += 72;
        //Step 3 multiply y by negative 1
        position.y = position.y * -1;
        //Step 4 add 72 to y
        position.y += 72;

        //Heading time!
        //Heading will start in degrees, it can also be negative, lets make sure no negatives
        if (yaw < 0) {
            yaw += 360;
        }

        //Convert yaw to the pedro coordinate version
        yaw -= 90;
        //Finally make radians

        return new Pose(position.x, position.y, Math.toRadians(yaw));
    }
    public Pose findAveragePose(ArrayList<Pose> poses){
        double averageX = 0;
        double averageY = 0;
        double averageHeading = 0;
        for (int i = 0; i < poses.size(); i++){
            averageX += poses.get(i).getX();
            averageY += poses.get(i).getY();
            averageHeading += poses.get(i).getHeading();
        }
        averageX /= poses.size();
        averageY /= poses.size();
        averageHeading /= poses.size();
        return new Pose(averageX,averageY,averageHeading);
    }
    public ArrayList<Pose> removeOutliers(ArrayList<Pose> poses){
       Pose averagePose = findAveragePose(poses);
        for (int i = 0; i < poses.size(); i++){
            double distanceX = Math.abs(averagePose.getX() - poses.get(i).getX());
            if (distanceX > distanceThreshold){
                poses.remove(i);
                continue;
            }
            double distanceY = Math.abs(averagePose.getY() - poses.get(i).getY());
            if (distanceY > distanceThreshold){
                poses.remove(i);
                continue;
            }
            double distanceHeading = Math.toDegrees(Math.abs(averagePose.getHeading() - poses.get(i).getHeading()));
            if (distanceHeading > headingThresholdDegrees){
                poses.remove(i);
                continue;
            }
        }
        return  poses;
    }
    public void relocalize(){
        relocalizeCooldownTimer.setWait(relocalizationCooldownTime);
        localizeSequence = 0;
        mode = Mode.RELOCALIZE;
        relocalizeOverideTimer.setWait(relocalizationOverrideTime);
    }
    public void autoRelocalize(Telemetry telemetry,FollowerHandler followerHandler){
        //Not already relocalizing
        telemetry.addData("MODE: ", mode);
        telemetry.addData("CAN RELOCALIZE",canRelocalize);
        telemetry.addData("VELOCITY WORKS",Math.abs(followerHandler.getFollower().getVelocity().getMagnitude()) > velocityCountsAsMovingThreshold);
        telemetry.addData("Cooldown ON", !relocalizeCooldownTimer.doneWaiting());
        telemetry.addData("Automatic Relocalization",automaticRelocalization);
        if (!automaticRelocalization){
            return;
        }
        if (!relocalizeCooldownTimer.doneWaiting()){
            return;
        }
        if (mode == Mode.RELOCALIZE){
            return;
        }
        //No reading or no tag in sight
        if (!canRelocalize){
            return;
        }
        //No moving
        if (Math.abs(followerHandler.getFollower().getVelocity().getMagnitude()) > velocityCountsAsMovingThreshold){
            return;
        }
        relocalize();
    }

    public void update(Telemetry telemetry, FollowerHandler followerHandler) {
        autoRelocalize(telemetry,followerHandler);
        if (localized == true && mode == Mode.RESTING){
            localized = false;
        }
        if (mode == Mode.RELOCALIZE){
            if (localizeSequence > numberOfLocalizationAttempts || relocalizeOverideTimer.doneWaiting() || Math.abs(followerHandler.getFollower().getVelocity().getMagnitude()) > velocityCountsAsMovingThreshold){
                mode = Mode.RESTING;
                localizeSequence = 0;
                localized = true;
                relocalizePose = findAveragePose(removeOutliers(relocalizePositions));
                relocalizePositions.clear();
                followerHandler.setStartingPose(getPose(followerHandler.getFollower().getPose()));


            }else{
                relocalizePositions.add(pedroPose);
                localizeSequence++;
            }
        }
        canRelocalize = false;
        LLResult result = limelight.getLatestResult();
        relocalizingId = 0;
        telemetry.addLine("LIMELIGHT -----------");
        telemetry.addData("LimelightMotif", LimeLight.motifToString(motif));
        telemetry.addData("Limelight Can relocalize", canRelocalize);
        telemetry.addData("Relocalize Id", relocalizingId);
        if (result == null || !result.isValid()) {
            return;
        }
        for (int i = 0; i < result.getFiducialResults().size(); i++) {
            LLResultTypes.FiducialResult ficidual = result.getFiducialResults().get(i);
            int id = ficidual.getFiducialId();
            if (motifId(id)) {
                setMotif(id);
                return;
            }

            //Should never happen unless somebody is wearing a giant april tag on their shirt
            if (id != redId && id != blueId) {
                return;
            }


            //can relocalize
            relocalizingId = id;
            canRelocalize = true;
            Pose3D botpose = result.getBotpose();

            Position poseIn = botpose.getPosition().toUnit(DistanceUnit.INCH);
            double yaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
            pedroPose = translateLimelightPoseToPedro(poseIn, yaw);


            telemetry.addData("LimelightMotif", LimeLight.motifToString(motif));
            telemetry.addData("Limelight Can relocalize", canRelocalize);
            telemetry.addData("Yaw", yaw);
            telemetry.addData("getBotPose", botpose);
            telemetry.addData("PoseIn", poseIn);
            telemetry.addData("Pedro Position", pedroPose);
            telemetry.addData("Relocalize Id", relocalizingId);
            telemetry.addData("Mode",mode);
            telemetry.addData("V Magnitude",followerHandler.getFollower().getVelocity().getMagnitude());
        }

    }
    public Pose getPose(Pose currentPose){
        if (!localized || !canRelocalize){
            return currentPose;
        }
        return relocalizePose;
    }
    public void ftcDashUpdate(TelemetryPacket telemetryPacket){
        telemetryPacket.put("Localized",localized);
        telemetryPacket.put("Can Relocalize",canRelocalize);
        telemetryPacket.put("Localize Sequence",localizeSequence);
        telemetryPacket.put("Relocalize Pose X",relocalizePose.getX());
        telemetryPacket.put("Relocalize Pose Y",relocalizePose.getY());
        telemetryPacket.put("Relocalize Pose Heading Degrees",Math.toDegrees(relocalizePose.getHeading()));
    }

}
