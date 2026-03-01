package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Inspector", group = "Test")
public class LimelightTelemetry extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Ensure your pipeline is set
        limelight.start();

        telemetry.addLine("Limelight Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // 1. TX: Horizontal Offset (What you've been using)
            double tx = result.getTx();

            // 2. TY: Vertical Offset (Excellent for estimating distance)
            double ty = result.getTy();

            // 3. TA: Target Area (0% to 100% of the screen)
            double ta = result.getTa();

            // 4. TS: Skew (Rotation of the target)

            telemetry.addLine("=== TARGET DETECTED ===");
            telemetry.addData("Horizontal (TX)", "%.2f°", tx);
            telemetry.addData("Vertical (TY)", "%.2f°", ty);
            telemetry.addData("Area (TA)", "%.2f%%", ta);

            // --- DISTANCE ESTIMATION TEST ---
            // As you move back, TY will decrease.
            // As you move forward, TY will increase.
            if (ty > 10) {
                telemetry.addLine("Status: Very Close");
            } else if (ty < -5) {
                telemetry.addLine("Status: Very Far");
            } else {
                telemetry.addLine("Status: Mid-Range");
            }

        } else {
            telemetry.addLine("### NO TARGET VISIBLE ###");
        }

        telemetry.update();
    }
}