package org.firstinspires.ftc.teamcode.Subsystems.Webcam;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

@Config
public class Webcam {

    public enum States {
        ON,
        OFF
    }

    ColorBlobLocatorProcessor purpleLocator, greenLocator;
    VisionPortal portal;
    public static double minArea = 50;
    public static double maxArea = 20000;
    public static double minCircularity = .2;
    public static double maxCircularity = 1;
    public static double intakeWidth = 16;
    public static int cameraWidth = 320;
    public static int cameraHeight = 240;
    public static double pixelsPerInch = 5.8;
    public static int dilation = 7;
    public static int blurSize= 5;

    public static double pixelsToInch(double pixels) {
        return pixels / pixelsPerInch;
    }

    public void initiate(HardwareMap hardwareMap) {
        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0.5, 1, -1))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(blurSize)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(dilation)       // Expand blobs to fill any divots on the edges
                .setErodeSize(dilation)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
        greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0.5, 1, -1))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
        portal = new VisionPortal.Builder()
                .addProcessors(purpleLocator,greenLocator)
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        FtcDashboard.getInstance().startCameraStream(portal, 0);

    }

    public double targetPixelDisplacement = 0;
    public double targetIncheDisplacement = 0;
    Cluster targetCluster;

    public double getTargetIncheDisplacement() {
        return targetIncheDisplacement;
    }

    public double getTargetInches(Lights.TeamColors teamColor) {
        if (teamColor == Lights.TeamColors.RED) {
            return targetIncheDisplacement * -1;
        }
        return targetIncheDisplacement;
    }

    public void off() {
        portal.close();
    }
    public void on(){
        portal.resumeStreaming();
    }

    ArrayList<Double> balls = new ArrayList<>();
    ArrayList<Cluster> clusters = new ArrayList<>();

    public void update(Telemetry telemetry) {
        if (!clusters.isEmpty()){
            clusters.clear();
        }
        if (!balls.isEmpty()){
            balls.clear();
        }
        //Make a list of blobs including purple and green
        List<ColorBlobLocatorProcessor.Blob> blobs = purpleLocator.getBlobs();
        blobs.addAll(greenLocator.getBlobs());

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                minArea, maxArea, blobs);  // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                minCircularity, maxCircularity, blobs);     /* filter out non-circular blobs.
         * NOTE: You may want to adjust the minimum value depending on your use case.
         * Circularity values will be affected by shadows, and will therefore vary based
         * on the location of the camera on your robot and venue lighting. It is strongly
         * encouraged to test your vision on the competition field if your event allows
         * sensor calibration time.
         */

        /*
         * The list of Blobs can be sorted using the same Blob attributes as listed above.
         * No more than one sort call should be made.  Sorting can use ascending or descending order.
         * Here is an example.:
         *   ColorBlobLocatorProcessor.Util.sortByCriteria(
         *      ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);
         */

        telemetry.addLine("Circularity Radius Center");

        // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
        int i = 0;
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            Circle circleFit = b.getCircle();
            balls.add((double) circleFit.getX());
            telemetry.addData("ball " + i, circleFit.getCenter());
            i++;
        }
        clusters = Cluster.createClusters(balls);

        telemetry.addData("Target Inches", targetIncheDisplacement);
        telemetry.addData("Target Pixels", targetPixelDisplacement);

        if (clusters.isEmpty()){
            return;
        }
        targetCluster = Cluster.getClosestCluster(Cluster.getBiggestCluster(clusters));
        targetPixelDisplacement =  targetCluster.getCenter() - (int) (cameraWidth / 2);
        targetIncheDisplacement = pixelsToInch(targetPixelDisplacement);


        Cluster.clusterStatus(telemetry,clusters);


    }
    public void updateFTCDashboard(TelemetryPacket telemetryPacket){
        telemetryPacket.put("TargetInches", targetIncheDisplacement);
    }
}
