package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "TurrentTracking", group = "Test")
public class TurrentTracking extends LinearOpMode {

    // Declare the filtered tx variable
    private double txFiltered = 0.0;  // Initialize the filtered tx value

    // Declare the filter strength for tx filtering
    private static final double TX_FILTER_ALPHA = 0.50;  // Increased filtering to smooth out the tx value

    // Declare the aiming offset for turret bias (increased to shift to the right)
    private static final double AIM_OFFSET_DEG_FAR = 2.0; // Right shift for far shots
    private static final double AIM_OFFSET_DEG_CLOSE = 1.0; // Slightly reduced bias for close shots

    // ====== DRIVE ======
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // ====== TURRET ======
    private DcMotorEx turret;

    // ====== OUTTAKE (shooter) ======
    private DcMotorEx outtake;

    // ====== INTAKE ======
    private DcMotor intake;

    // ====== SERVOS ======
    private Servo kick1, kick2, kick3;

    private static final double KICK_REST_POS = 0.0;
    private static final double KICK_FIRE_POS = 1.0;

    // ====== IMU ======
    private IMU imu;
    private static final double HEADING_SIGN = -1.0; // flip if heading feels backwards

    private double headingFiltered = 0.0;
    private static final double HEADING_FILTER_ALPHA = 0.30;

    // ====== TURRET ENCODER → DEGREES ======
    private static final double TICKS_PER_DEGREE = 4.44;
    private static final double TURRET_SIGN = 1.0; // flip to -1.0 if turret angle inverted

    // ====== WORLD LOCK (IMU fallback) ======
    private double worldTargetAngle = 0.0;
    private boolean hasWorldTarget = false;

    private static final double K_AIM_WORLD = 0.06;
    private static final double MAX_TURRET_POWER_WORLD = 0.75;
    private static final double ANGLE_DEADBAND_DEG = 2.0;

    // ====== LIMELIGHT ======
    private Limelight3A limelight;
    private static final int TARGET_TAG_ID = 24;

    // Limelight PID Control constants (smaller KP for smoother movement)
    private static final double LIMELIGHT_KP_TURN_FAR = 0.02;  // Far shot adjustment (higher sensitivity)
    private static final double LIMELIGHT_KP_TURN_CLOSE = 0.01; // Close shot adjustment (lower sensitivity)
    private static final double LIMELIGHT_KI_TURN = 0.0;   // Integral constant (not used in this case)
    private static final double LIMELIGHT_KD_TURN = 0.0;   // Derivative constant (not used in this case)
    private static final double LIMELIGHT_MAX_TURN = 0.70;  // Max turn power

    private static final double LIMELIGHT_AIM_TOLERANCE = 2.0; // Tolerance for aiming

    // ====== SHOOTER (pose-based distance → RPM) ======
    private static final double TICKS_PER_REV = 28.0; // goBILDA commonly 28
    private static final double G = 9.80665;

    private static final double RPM_MIN = 600;
    private static final double RPM_MAX = 4500;

    // geometry
    private static final double WHEEL_DIAMETER_IN = 4.0;
    private static final double SHOOTER_HEIGHT_IN = 17.5;
    private static final double GOAL_ENTRY_IN = 43.0;
    private static final double SHOOTER_ANGLE_DEG = 60.0;

    // Lower = commands MORE rpm (stronger).
    private static final double SHOOTER_EFF = 0.55;

    // ====== LIVE TUNE RPM SCALE ======
    private double rpmScale = 1.68;

    // last known distance (inches) from pose, for when tag flickers
    private double lastDistanceIn = 0.0;
    private double lastRpmCmd = 0.0;

    // debug: what ID did we actually use this frame?
    private int lastUsedFiducialId = -1;

    // dpad edge detect
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Define close and far power scaling factors
    private static final double FAR_RPM_SCALE = 1.68;
    private static final double CLOSE_RPM_SCALE = 1.45; // Adjust this value to make close shots weaker

    // PID variables for turret aiming
    private double previousError = 0.0;  // Previous error for derivative calculation
    private double integral = 0.0;       // Integral term (used for error accumulation)

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== DRIVE =====
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== TURRET =====
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== OUTTAKE =====
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ===== INTAKE =====
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // ===== SERVOS =====
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick3 = hardwareMap.get(Servo.class, "kick3");

        // ===== IMU =====
        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
        headingFiltered = getSignedHeading();

        // ===== LIMELIGHT =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // pipeline 0 must be AprilTags
        limelight.start();

        telemetry.addLine("TurrentTracking: ID24 only");
        telemetry.addLine("LT aim | Lost tag -> IMU world lock");
        telemetry.addLine("RT shoot | A intake | Y/B/X servos");
        telemetry.addLine("DpadUp/Down = RPM scale (hold DpadLeft for bigger steps)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== LIVE RPM SCALE TUNING (edge-based) =====
            boolean dUp = gamepad1.dpad_up;
            boolean dDown = gamepad1.dpad_down;

            double step = gamepad1.dpad_left ? 0.10 : 0.02; // big vs small steps

            if (dUp && !lastDpadUp) rpmScale += step;
            if (dDown && !lastDpadDown) rpmScale -= step;

            rpmScale = clamp(rpmScale, 0.50, 3.00);

            lastDpadUp = dUp;
            lastDpadDown = dDown;

            // ===== IMU filtered heading =====
            double rawHeading = getSignedHeading();
            double dHead = wrapAngle(rawHeading - headingFiltered);
            headingFiltered = wrapAngle(headingFiltered + HEADING_FILTER_ALPHA * dHead);
            double heading = headingFiltered;

            double turretDeg = getTurretDeg();

            // ===== drive =====
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;
            driveRobot(drive, strafe, turn);

            // ===== intake =====
            intake.setPower(gamepad1.a ? 1.0 : 0.0);

            // ===== servos (your exact mapping) =====
            kick1.setPosition(gamepad1.y ? 0.7: 0.1);
            kick2.setPosition(gamepad1.b ? 0.2 : 0.95);  // Adjusted kicking position
            kick3.setPosition(gamepad1.x ? 0.45 : 1);

            // ===== Limelight: ONLY ID 24 =====
            LLResult result = limelight.getLatestResult();
            boolean hasTag24Now = false;
            double txNow = 0.0;
            double distInNow = lastDistanceIn;
            int usedId = -1;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                if (fids != null && !fids.isEmpty()) {
                    for (LLResultTypes.FiducialResult fid : fids) {
                        int id = fid.getFiducialId();
                        if (id == TARGET_TAG_ID) {
                            hasTag24Now = true;

                            txNow = fid.getTargetXDegrees();

                            Pose3D pose = fid.getRobotPoseTargetSpace();
                            double zMeters = Math.abs(pose.getPosition().z);
                            distInNow = zMeters * 39.3701;

                            break;
                        }
                    }
                }
            }

            lastUsedFiducialId = usedId;

            if (hasTag24Now) {
                lastDistanceIn = distInNow;

                // tx filter
                txFiltered += TX_FILTER_ALPHA * (txNow - txFiltered);

                // Update world lock every time we see ID24
                worldTargetAngle = wrapAngle(heading + turretDeg);
                hasWorldTarget = true;
            }

            // ===== turret control =====
            boolean autoAim = gamepad1.left_trigger > 0.2;
            double turretPower = 0.0;
            String mode;

            if (autoAim) {
                if (hasTag24Now) {
                    double error = txFiltered + (lastDistanceIn > 24 ? AIM_OFFSET_DEG_FAR : AIM_OFFSET_DEG_CLOSE);

                    // PID control
                    integral += error;
                    double derivative = error - previousError;
                    turretPower = LIMELIGHT_KP_TURN_FAR * error + LIMELIGHT_KI_TURN * integral + LIMELIGHT_KD_TURN * derivative;

                    // Store previous error for next iteration
                    previousError = error;

                    // If error is within tolerance, stop moving turret
                    if (Math.abs(error) < LIMELIGHT_AIM_TOLERANCE) {
                        turretPower = 0.0;
                    }

                    turretPower = clamp(turretPower, -LIMELIGHT_MAX_TURN, LIMELIGHT_MAX_TURN);
                    mode = "LL tracking ID24 (PID)";
                } else if (hasWorldTarget) {
                    double turretWorldNow = wrapAngle(heading + turretDeg);
                    double errorWorld = wrapAngle(worldTargetAngle - turretWorldNow);

                    if (Math.abs(errorWorld) < ANGLE_DEADBAND_DEG) {
                        turretPower = 0.0;
                    } else {
                        turretPower = K_AIM_WORLD * errorWorld;
                        turretPower = clamp(turretPower, -MAX_TURRET_POWER_WORLD, MAX_TURRET_POWER_WORLD);
                    }

                    mode = "IMU world lock";
                } else {
                    turretPower = 0.0;
                    mode = "Auto, no lock yet";
                }
            } else {
                double bump = 0.4;
                if (gamepad1.left_bumper) turretPower = -bump;
                else if (gamepad1.right_bumper) turretPower = bump;
                else turretPower = 0.0;

                mode = "Manual";
            }

            turret.setPower(turretPower);

            // ===== shooter velocity from pose distance =====
            boolean shoot = gamepad1.right_trigger > 0.2;
            if (shoot && hasTag24Now) {  // Only shoot when the tag is detected
                double rpm = computeRpmFromDistanceInches(lastDistanceIn, rpmScale);
                // Adjust based on the distance:
                if (lastDistanceIn > 24) {  // Example: Far shot
                    rpmScale = FAR_RPM_SCALE;
                } else {  // Close shot
                    rpmScale = CLOSE_RPM_SCALE;
                }
                rpm = computeRpmFromDistanceInches(lastDistanceIn, rpmScale);
                rpm = clamp(rpm, RPM_MIN, RPM_MAX);
                lastRpmCmd = rpm;

                double ticksPerSec = rpmToTicksPerSecond(rpm, TICKS_PER_REV);
                outtake.setVelocity(ticksPerSec);
            } else {
                outtake.setVelocity(0.0);  // Stop shooter when trigger is not pressed or tag is not visible
            }

            // ===== telemetry =====
            telemetry.addData("Mode", mode);
            telemetry.addData("UsedFiducialId", lastUsedFiducialId);
            telemetry.addData("HasTag24Now", hasTag24Now);
            telemetry.addData("txFiltered", txFiltered);
            telemetry.addData("DistIn (pose Z)", lastDistanceIn);

            telemetry.addData("RPM_SCALE", rpmScale);
            telemetry.addData("RPM cmd", lastRpmCmd);

            telemetry.addData("TurretPower", turretPower);
            telemetry.update();
        }

        driveRobot(0, 0, 0);
        turret.setPower(0.0);
        outtake.setVelocity(0.0);
        intake.setPower(0.0);
    }

    // ===== shooter math =====
    private double computeRpmFromDistanceInches(double xIn, double rpmScale) {
        double x = xIn * 0.0254;
        double dy = (GOAL_ENTRY_IN - SHOOTER_HEIGHT_IN) * 0.0254;

        double theta = Math.toRadians(SHOOTER_ANGLE_DEG);
        double tan = Math.tan(theta);
        double cos = Math.cos(theta);

        double denom = (x * tan) - dy;
        if (denom <= 0.001) return RPM_MAX;

        // required BALL exit speed (m/s)
        double v = Math.sqrt((G * x * x) / (2.0 * cos * cos * denom));

        // wheel circumference
        double wheelDiamM = WHEEL_DIAMETER_IN * 0.0254;
        double wheelCircM = Math.PI * wheelDiamM;

        // convert to wheel rpm + tuning scale
        double wheelRps = (v / wheelCircM) / SHOOTER_EFF;
        double rpm = wheelRps * 60.0;

        return rpm * rpmScale;
    }

    private double rpmToTicksPerSecond(double rpm, double tpr) {
        return rpm * tpr / 60.0;
    }

    // ===== helpers =====
    private double getSignedHeading() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return HEADING_SIGN * wrapAngle(yaw);
    }

    private double getTurretDeg() {
        return (turret.getCurrentPosition() / TICKS_PER_DEGREE) * TURRET_SIGN;
    }

    private double wrapAngle(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void driveRobot(double drive, double strafe, double turn) {
        double lf = drive + strafe + turn;
        double rf = drive - strafe - turn;
        double lb = drive - strafe + turn;
        double rb = drive + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);
    }
}
