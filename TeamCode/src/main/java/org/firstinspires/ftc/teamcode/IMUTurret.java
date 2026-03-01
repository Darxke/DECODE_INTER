package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "IMU Turret Tracking - LIVE", group = "Production")
public class IMUTurret extends OpMode {

    private DcMotorEx turretMotor;
    private IMU imu;

    // --- CALIBRATED MATH ---
    private final double TICKS_PER_MOTOR_REV = 384.5; // 435 RPM
    private final double CALIBRATED_RATIO = 4.075;    // Your tested magic number
    private final double TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * CALIBRATED_RATIO;

    // PID Variables - Start gentle!
    private double kP = 0.035;
    private double kD = 0.003;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // targetHeading 0.0 means the turret will try to point at whatever
    // direction the robot was facing when you hit "Init".
    private double targetHeading = 0.0;

    private final double MAX_POWER = 0.45;
    private final double MIN_POWER = 0.15; // Adjusted slightly for the 4.075 ratio friction

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // You have REVERSE here; this should make "Left" power move the turret "Left"
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        imu.resetYaw();
        timer.reset();
    }

    @Override
    public void loop() {
        // 1. Get current headings
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Calculate where the turret is relative to the robot chassis
        double turretDegrees = (turretMotor.getCurrentPosition() / TICKS_PER_TURRET_REV) * 360.0;

        // Calculate where the turret is pointing in the WORLD
        double currentWorldHeading = robotHeading + turretDegrees;

        // 2. PID Logic
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 0.001;

        // The goal is for WORLD HEADING to equal TARGET HEADING
        double error = targetHeading - currentWorldHeading;

        // Wrap error (-180 to 180) to prevent the "long way around" spin
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;

        double derivative = (error - lastError) / dt;
        double pidOutput = (error * kP) + (derivative * kD);

        // 3. Deadband and Minimum Power
        // If error is less than 1 degree, stop trying (prevents buzzing)
        if (Math.abs(error) > 1.0) {
            if (Math.abs(pidOutput) < MIN_POWER) {
                pidOutput = Math.signum(pidOutput) * MIN_POWER;
            }
        } else {
            pidOutput = 0;
        }

        // 4. Final Output
        double motorPower = Range.clip(pidOutput, -MAX_POWER, MAX_POWER);
        turretMotor.setPower(motorPower);

        lastError = error;

        // Telemetry
        telemetry.addData("World Heading (Pointer)", "%.2f°", currentWorldHeading);
        telemetry.addData("Robot Yaw", "%.2f°", robotHeading);
        telemetry.addData("Turret Rel Angle", "%.2f°", turretDegrees);
        telemetry.addData("Error", "%.2f°", error);
        telemetry.addData("Motor Power", "%.3f", motorPower);
        telemetry.update();
    }
}