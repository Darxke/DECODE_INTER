package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TurrentTracking", group = "Test")
public class TurrentTracking extends LinearOpMode {

    // Turret motor (312 rpm, 48-20-200 gear train)
    private DcMotorEx turret;

    // Drive motors - make sure these names match your RC config
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // IMU - check the name in your config
    private IMU imu;

    // ===== PID for turret =====
    // Slightly calmer since ticks/deg is larger now
    private double kP = 0.0025;
    private double kI = 0.0;
    private double kD = 0.0008;

    private double integral = 0;
    private double lastError = 0;

    // Separate timers so PID dt and target dt do not fight each other
    private double lastTimePID = 0;
    private double lastTimeTarget = 0;

    // 312 rpm motor, 48-20-200 gear: 2240 ticks / 360 deg ≈ 6.22 ticks/deg
    private final double TICKS_PER_DEGREE = 6.22;

    // Soft limits in encoder ticks - change to safe physical limits
    // 6.22 ticks/deg → 1500 ticks ≈ 241 deg
    private int TURRET_MIN = -1500;
    private int TURRET_MAX = 1500;

    // Desired turret angle relative to ROBOT, in degrees (-180 to 180)
    private double turretTargetDegrees = 0;

    // Direction of the goal in FIELD coordinates (degrees, -180 to 180)
    private double goalHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ====== HARDWARE MAP ======
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        imu = hardwareMap.get(IMU.class, "imu");

        // Reset yaw so 0 is "starting direction"
        imu.resetYaw();

        // One side reversed so forward is actually forward (flip sides if needed)
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastTimePID = getRuntime();
        lastTimeTarget = getRuntime();

        telemetry.addLine("TurrentTracking ready (Gamepad 1 only)");
        telemetry.update();

        waitForStart();

        // At the start, assume goal is "straight ahead"
        goalHeading = wrapAngle(getHeading());
        turretTargetDegrees = 0;

        while (opModeIsActive()) {

            // ================== DRIVING (GAMEPAD 1) ==================
            // Forward/back OK, strafe fixed, turn inverted to feel normal.
            double y = gamepad1.left_stick_y;        // forward/back
            double x = -gamepad1.left_stick_x;       // LEFT stick left = strafe left
            double turn = -gamepad1.right_stick_x;   // invert turn

            double lfPower = y + x + turn;
            double rfPower = y - x - turn;
            double lbPower = y - x + turn;
            double rbPower = y + x - turn;

            leftFront.setPower(lfPower);
            rightFront.setPower(rfPower);
            leftBack.setPower(lbPower);
            rightBack.setPower(rbPower);

            // ===== PRESET TURRET POSITIONS FOR DEBUG =====
            // X = -90 deg, Y = +90 deg (simple way to check if turret moves at all)
            if (gamepad1.x) {
                turretTargetDegrees = -90;
            } else if (gamepad1.y) {
                turretTargetDegrees = 90;
            }

            // ================== SET GOAL DIRECTION ==================
            // Press A to say "goal is where the TURRET is looking right now"
            if (gamepad1.a) {
                double robotHeading = wrapAngle(getHeading());
                double turretRelDeg = turret.getCurrentPosition() / TICKS_PER_DEGREE;
                // world angle = robot heading + turret angle relative to robot
                goalHeading = wrapAngle(robotHeading + turretRelDeg);
                // keep target equal to current turret angle so it doesn't jump
                turretTargetDegrees = turretRelDeg;
            }

            // ================== AUTO TRACK OR MANUAL ==================
            boolean autoTrack = gamepad1.left_trigger > 0.2;

            if (autoTrack) {
                // AUTO: turret keeps facing goal direction in field frame
                double robotHeading = wrapAngle(getHeading());

                // desired turret angle relative to robot
                double desiredTurretRel = wrapAngle(goalHeading - robotHeading);
                turretTargetDegrees = desiredTurretRel;

                runTurretPID();
            } else {
                // MANUAL: bump turret using bumpers, still using PID
                double dtTarget = getDeltaTimeForTarget();

                double manualSpeedDegPerSec = 90; // how fast turret target moves with bumper held

                if (gamepad1.left_bumper) {
                    turretTargetDegrees -= manualSpeedDegPerSec * dtTarget;
                } else if (gamepad1.right_bumper) {
                    turretTargetDegrees += manualSpeedDegPerSec * dtTarget;
                }

                // clamp target angle to match soft limits
                double minDeg = TURRET_MIN / TICKS_PER_DEGREE;
                double maxDeg = TURRET_MAX / TICKS_PER_DEGREE;

                if (turretTargetDegrees < minDeg) turretTargetDegrees = minDeg;
                if (turretTargetDegrees > maxDeg) turretTargetDegrees = maxDeg;

                runTurretPID();
            }

            // ================== TELEMETRY ==================
            double turretRelDegNow = turret.getCurrentPosition() / TICKS_PER_DEGREE;

            telemetry.addData("Heading (raw)", getHeading());
            telemetry.addData("Heading (wrapped)", wrapAngle(getHeading()));
            telemetry.addData("Goal Heading", goalHeading);
            telemetry.addData("Turret Target Deg (rel)", turretTargetDegrees);
            telemetry.addData("Turret Actual Deg (rel)", turretRelDegNow);
            telemetry.addData("Turret Pos Ticks", turret.getCurrentPosition());
            telemetry.addData("Auto Track", autoTrack);
            telemetry.update();
        }
    }

    // Run PID loop to drive turret toward turretTargetDegrees (relative to robot)
    private void runTurretPID() {
        int targetTicks = (int) (turretTargetDegrees * TICKS_PER_DEGREE);

        // clamp to soft limits
        if (targetTicks < TURRET_MIN) targetTicks = TURRET_MIN;
        if (targetTicks > TURRET_MAX) targetTicks = TURRET_MAX;

        int current = turret.getCurrentPosition();
        double error = targetTicks - current;

        double now = getRuntime();
        double dt = now - lastTimePID;
        if (dt <= 0) dt = 0.01;
        lastTimePID = now;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = kP * error + kI * integral + kD * derivative;

        // clamp turret power so it is strong but not insane
        double maxPower = 0.8;
        if (power > maxPower) power = maxPower;
        if (power < -maxPower) power = -maxPower;

        // extra safety on soft limits
        if ((current <= TURRET_MIN && power < 0) ||
                (current >= TURRET_MAX && power > 0)) {
            power = 0;
        }

        turret.setPower(power);
    }

    // dt for adjusting the target angle manually
    private double getDeltaTimeForTarget() {
        double now = getRuntime();
        double dt = now - lastTimeTarget;
        if (dt <= 0) dt = 0.01;
        lastTimeTarget = now;
        return dt;
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        lastTimePID = getRuntime();
    }

    // Raw IMU yaw in degrees (-180 to +180)
    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Wrap any angle to range (-180, 180]
    private double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}
