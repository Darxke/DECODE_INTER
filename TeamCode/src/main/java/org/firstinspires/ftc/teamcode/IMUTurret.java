package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "IMU Turret Tracking", group = "Test")
public class IMUTurret extends OpMode {

    private DcMotorEx turretMotor;
    private IMU imu;

    // PID Variables
    private double kP = 0.035;
    private double kD = 0.003;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // The angle you want the turret to face (0 is straight ahead)
    private double targetHeading = 0.0;

    private final double MAX_POWER = 0.4;
    private final double MIN_POWER = 0.12;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Adjust these to match how your Control Hub is mounted!
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        imu.resetYaw(); // Sets current heading to 0
    }

    @Override
    public void loop() {
        // 1. Get current heading from IMU (-180 to 180 degrees)
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // 2. Calculate Timing
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 0.001;

        // 3. PID Logic
        // Error = Target - Current
        double error = targetHeading - currentHeading;

        // Wrap error to keep it between -180 and 180 (prevents 360-degree spins)
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;

        double derivative = (error - lastError) / dt;
        double pidOutput = (error * kP) + (derivative * kD);

        // 4. Apply Minimum Power (Static Friction)
        if (Math.abs(error) > 1.5) { // Deadband of 1.5 degrees
            if (Math.abs(pidOutput) < MIN_POWER) {
                pidOutput = Math.signum(pidOutput) * MIN_POWER;
            }
        } else {
            pidOutput = 0;
        }

        // 5. Output to Motor
        double motorPower = Range.clip(pidOutput, -MAX_POWER, MAX_POWER);
        turretMotor.setPower(motorPower);

        lastError = error;

        // Telemetry
        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("Current Heading", "%.2f", currentHeading);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Motor Power", "%.3f", motorPower);
        telemetry.update();
    }
}