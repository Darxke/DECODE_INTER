package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "GetTx", group = "Test")
public class GetTx extends OpMode {

    private DcMotorEx turretMotor;
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {
        // 1. Motor Setup (For encoder reading only)
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Motor power is NEVER set in this OpMode for safety

        // 2. Limelight Setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // 3. IMU Setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        telemetry.addLine("SAFETY MODE: Motor Power Disabled");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- VISION DATA ---
        LLResult result = limelight.getLatestResult();
        double tx = 0;
        boolean targetVisible = false;

        if (result != null && result.isValid()) {
            tx = result.getTx();
            targetVisible = true;
        }

        // --- HEADING DATA ---
        // Robot Heading (from Control Hub IMU)
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Turret Heading (from Motor Encoder)
        int encoderTicks = turretMotor.getCurrentPosition();
        // Assuming a GoBILDA 5202 19.2:1 motor (537.7 ticks per rev)
        // If your turret has a gear ratio (e.g. 2:1), multiply 537.7 by that ratio.
        double turretDegrees = (encoderTicks / 537.7) * 360.0;

        // --- TELEMETRY OUTPUT ---
        telemetry.addLine("=== VISION (LIMELIGHT) ===");
        telemetry.addData("Target Visible", targetVisible ? "YES" : "NO");
        telemetry.addData("Raw TX", "%.2f°", tx);
        telemetry.addData("Offset Error (vs -5.84)", "%.2f°", (tx - (-5.84)));

        telemetry.addLine("\n=== HEADINGS ===");
        telemetry.addData("Robot (IMU) Yaw", "%.2f°", robotHeading);
        telemetry.addData("Turret (Encoder) Deg", "%.2f°", turretDegrees);
        telemetry.addData("Ticks", encoderTicks);

        telemetry.addLine("\n=== MANUAL TEST ===");
        telemetry.addLine("Manually rotate the turret by hand.");
        telemetry.addLine("Check if 'Turret Deg' matches your physical move.");

        telemetry.update();
    }
}