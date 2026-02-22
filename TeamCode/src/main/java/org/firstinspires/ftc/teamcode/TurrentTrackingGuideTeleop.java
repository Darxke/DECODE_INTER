package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LimelightTurretTracking")
public class TurrentTrackingGuideTeleop extends OpMode {

    // Initialize your tracking guide and the Limelight
    private TurrentTrackingGuide turret = new TurrentTrackingGuide();
    Limelight3A limelight;
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 0;
    @Override
    public void init() {
        // Initialize the Turret class
        turret.init(hardwareMap);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        telemetry.addLine("Initialized all mechanisms");
    }

    @Override
    public void start() {
        turret.resetTimer();
        // Optional: switch to a specific pipeline (e.g., 0 for AprilTags, 1 for Color)
        limelight.pipelineSwitch(0);
    }

    // Define these at the top of your class


    @Override
    public void loop() {
        // --- TUNING LOGIC ---

        // B button cycles through step sizes (0.001, 0.01, etc.)
        if (gamepad1.backWasPressed()) { // Using standard boolean check; use your specific 'wasPressed' method if available
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // D-pad Left/Right adjusts the Proportional gain (kP)
        if (gamepad1.dpadLeftWasPressed()) {
            turret.setKp(turret.getKp() - stepSizes[stepIndex]);
        }
        if (gamepad1.dpadRightWasPressed()) {
            turret.setKp(turret.getKp() + stepSizes[stepIndex]);
        }

        // D-pad Up/Down adjusts the Derivative gain (kD)
        if (gamepad1.dpadUpWasPressed()) {
            turret.setkD(turret.getKD() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            turret.setkD(turret.getKD() - stepSizes[stepIndex]);
        }

        // --- VISION LOGIC ---
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            turret.update(result.getTx());
            telemetry.addData("Status", "Tracking");
        } else {
            turret.update(0);
            telemetry.addData("Status", "No Target");
        }

        // --- TELEMETRY ---
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addData("Current kP", turret.getKp());
        telemetry.addData("Current kD", turret.getKD());
        telemetry.update();
    }
}