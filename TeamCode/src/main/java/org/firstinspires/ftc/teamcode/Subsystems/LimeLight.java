package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class LimeLight {
    public enum BallColors {
        P,
        G,
        NONE
    }

    public static Pose redGoalPos = new Pose(128.5446293494705, 132.61119515885022, Math.toRadians(40));
    public static Pose blueGoalPos = new Pose(16, 132.61119515885022, Math.toRadians(140));

    static BallColors[] motif = {BallColors.P, BallColors.P, BallColors.G};

    public BallColors[] getMotif() {
        return motif;
    }

    public static final int blueId = 20;
    public static final int redId = 24;
    public static int GPPId = 21;
    public static int PGPId = 22;
    public static int PPGId = 23;

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

    public boolean canRelocalize = false;
    int relocalizingId = 0;
    Limelight3A limelight;

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


    public void update(Telemetry telemetry) {

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
        }

    }

    public Pose relocalize(Pose currentPose) {
        if (!canRelocalize) {
            return currentPose;
        }
        return pedroPose;
    }


    public void ftcDashUpdate(TelemetryPacket telemetryPacket) {

    }

}
