package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name = "AutoLimelight", group = "Auto")
public class AutoLimelight extends LinearOpMode {

    private Limelight3A limelight;

    // Set this to the pipeline index on your Limelight that is configured for AprilTags
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    // Optional: only accept these IDs. If you want "any ID", set this to an empty array.
    private static final int[] ALLOWED_IDS = {20, 24, 30};

    @Override
    public void runOpMode() {

        // IMPORTANT: your Control Hub config name must match this string (usually "limelight")
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);      // must be called before start()
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        int chosenId = -1;

        // "Init loop": keep updating telemetry before you press start
        while (!isStarted() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();

            chosenId = pickBestAllowedId(result, ALLOWED_IDS);

            telemetry.addData("LL Connected", limelight.isConnected());
            telemetry.addData("Chosen ID", chosenId);
            telemetry.addData("Seen IDs", listSeenIds(result));
            telemetry.update();

            sleep(20);
        }

        waitForStart();

        // Example branching in auto
        if (chosenId == 20) {
            // TODO: run path A
        } else if (chosenId == 24) {
            // TODO: run path B
        } else if (chosenId == 30) {
            // TODO: run path C
        } else {
            // TODO: default path when no allowed tag is seen
        }

        limelight.stop();
    }

    /**
     * Picks the "best" tag ID from the current Limelight result.
     * This version chooses the allowed tag with the largest target area (closest / biggest in view).
     */
    private static int pickBestAllowedId(LLResult result, int[] allowedIds) {
        if (result == null || !result.isValid()) return -1;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return -1;

        LLResultTypes.FiducialResult best = null;
        double bestArea = -1;

        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();

            if (allowedIds.length > 0 && !contains(allowedIds, id)) continue;

            double area = tag.getTargetArea();  // percent of image
            if (area > bestArea) {
                bestArea = area;
                best = tag;
            }
        }

        return (best == null) ? -1 : best.getFiducialId();
    }

    private static String listSeenIds(LLResult result) {
        if (result == null || !result.isValid()) return "(none)";

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return "(none)";

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < tags.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(tags.get(i).getFiducialId());
        }
        return sb.toString();
    }

    private static boolean contains(int[] arr, int value) {
        for (int x : arr) if (x == value) return true;
        return false;
    }
}
