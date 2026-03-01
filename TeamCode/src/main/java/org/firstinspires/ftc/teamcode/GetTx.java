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

    // --- MATH CONSTANTS ---
    // GoBILDA 435 RPM motor = 384.5 ticks per rev
    private final double TICKS_PER_MOTOR_REV = 384.5;

    // We adjusted this to 3.125 because your 6.25 ratio was yielding half-scale (45 instead of 90)
    private final double EFFECTIVE_RATIO = 4.075;
    private final double TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * EFFECTIVE_RATIO;

    @Override
    public void init() {
        // 1. Motor Setup (Encoder only)
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // 2. Limelight Setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // 3. IMU Setup with your specific orientation
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        imu.resetYaw();

        telemetry.addLine("TELEMETRY MODE: No Motor Movement");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- VISION DATA ---
        LLResult result = limelight.getLatestResult();
        double tx = (result != null && result.isValid()) ? result.getTx() : 0;

        // --- HEADING DATA ---
        // 1. Robot Heading from IMU
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // 2. Turret Heading from Encoder
        // We add a negative sign because you said turning left was currently negative
        int rawTicks = turretMotor.getCurrentPosition();
        double turretDegrees = -(rawTicks / TICKS_PER_TURRET_REV) * 360.0;

        // 3. World Heading (Where turret is pointing relative to the field)
        double worldHeading = robotHeading + turretDegrees;

        // --- TELEMETRY OUTPUT ---
        telemetry.addLine("=== SYSTEM CHECK (TURN LEFT TO TEST) ===");

        // Both should increase (go positive) when you turn left
        telemetry.addData("1. Robot (IMU) Yaw", "%.2f°", robotHeading);
        telemetry.addData("2. Turret (Encoder) Deg", "%.2f°", turretDegrees);

        telemetry.addLine("\n=== VISION ===");
        telemetry.addData("Target Visible", (result != null && result.isValid()));
        telemetry.addData("Raw TX", "%.2f°", tx);
        telemetry.addData("Target Offset Error", "%.2f°", (tx - (-5.84)));

        telemetry.addLine("\n=== ADVANCED ===");
        telemetry.addData("World Heading", "%.2f°", worldHeading);
        telemetry.addData("Raw Encoder Ticks", rawTicks);

        telemetry.update();
    }
}