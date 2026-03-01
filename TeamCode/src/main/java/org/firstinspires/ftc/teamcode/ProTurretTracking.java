package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Turret: Anti-Wrap Shortest Path", group = "Production")
public class ProTurretTracking extends OpMode {

    private DcMotorEx turretMotor;
    private IMU imu;
    private Limelight3A limelight;

    // --- HARDWARE CALIBRATION ---
    private final double TICKS_PER_MOTOR_REV = 384.5;
    private final double CALIBRATED_RATIO = 4.075;
    private final double TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * CALIBRATED_RATIO;

    // --- CAMERA OFFSET ---
    // RAW TX when perfectly aimed at target - calibrate this first
    private final double TARGET_TX = -8.3;

    // --- ANTI-WRAP SETTINGS ---
    private final double SAFE_LIMIT = 175.0;

    // --- PID TUNING ---
    private double kP_Vision = 0.035;
    private double kP_IMU = 0.035;
    private final double MAX_POWER = 0.85;
    private final double MIN_POWER = 0.05;
    private final double VISION_DEADBAND = 1.0;

    private double targetWorldHeading = 0.0;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        targetWorldHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addData("Status", "Initialized - ENSURE TURRET IS CENTERED");
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double turretDegrees = (turretMotor.getCurrentPosition() / TICKS_PER_TURRET_REV) * 360.0;
        double currentWorldHeading = robotHeading + turretDegrees;

        double error;
        String mode;

        if (result != null && result.isValid() && result.getTa() > 0.1) {
            mode = "VISION LOCK";

            // How far TX is from where it should be when perfectly aimed
            error = -(result.getTx() - TARGET_TX);

            // Only update world target when stable and locked
            // This prevents the turret chasing itself when robot rotates
            if (Math.abs(error) < VISION_DEADBAND) {
                targetWorldHeading = currentWorldHeading;
            }

        } else {
            mode = "IMU FALLBACK";

            // Shortest path to saved world target
            double targetRelative = normalizeAngle(targetWorldHeading - robotHeading);
            error = normalizeAngle(targetRelative - turretDegrees);

            // Wire protector - if shortest path crosses the back, go the long way instead
            if (Math.abs(turretDegrees + error) > SAFE_LIMIT) {
                if (error > 0) error -= 360;
                else error += 360;
                mode = "UNWINDING";
            }
        }

        // Calculate motor output
        double pidOutput = error * kP_Vision;

        // Apply MIN_POWER floor so motor doesn't stall
        if (Math.abs(error) > VISION_DEADBAND) {
            if (Math.abs(pidOutput) < MIN_POWER) {
                pidOutput = Math.signum(pidOutput) * MIN_POWER;
            }
        } else {
            pidOutput = 0;
        }

        // Hard safety clamps at physical limits
        if (turretDegrees >= SAFE_LIMIT && pidOutput > 0) pidOutput = 0;
        if (turretDegrees <= -SAFE_LIMIT && pidOutput < 0) pidOutput = 0;

        turretMotor.setPower(Range.clip(pidOutput, -MAX_POWER, MAX_POWER));

        // Manual re-zero if turret gets bumped
        if (gamepad1.start) {
            turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // --- TELEMETRY ---
        telemetry.addData("MODE", mode);
        telemetry.addData("Turret Pos", "%.1f°", turretDegrees);
        telemetry.addData("Target World", "%.1f°", targetWorldHeading);
        telemetry.addData("Error", "%.2f°", error);
        telemetry.addData("Motor Power", "%.3f", Range.clip(pidOutput, -MAX_POWER, MAX_POWER));
        if (result != null && result.isValid()) {
            telemetry.addData("RAW TX", "%.2f", result.getTx());
            telemetry.addData("Target Area", "%.3f", result.getTa());
        }
        telemetry.update();
    }
}