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

    // ====== IMU ======
    private IMU imu;
    private static final double HEADING_SIGN = -1.0;
    private double headingFiltered = 0.0;
    private static final double HEADING_FILTER_ALPHA = 0.30;

    // ====== TURRET ENCODER → DEGREES ======
    private static final double TICKS_PER_DEGREE = 4.44;
    private static final double TURRET_SIGN = 1.0;

    // ====== WORLD LOCK (IMU fallback) ======
    private double worldTargetAngle = 0.0;
    private boolean hasWorldTarget = false;

    private static final double K_AIM_WORLD = 0.10;
    private static final double MAX_TURRET_POWER_WORLD = 0.60;
    private static final double ANGLE_DEADBAND_DEG = 0.6;
    private static final double WORLD_MIN_POWER = 0.07;

    // ====== LIMELIGHT ======
    private Limelight3A limelight;
    private static final int TARGET_TAG_ID = 24;

    // Tracking gains (calmer than before)
    private static final double LIMELIGHT_KP_TURN_FAR = 0.012;
    private static final double LIMELIGHT_KP_TURN_CLOSE = 0.009;
    private static final double LIMELIGHT_KD = 0.010;
    private static final double LIMELIGHT_MAX_TURN = 0.75;

    private static final double LIMELIGHT_AIM_TOLERANCE = 0.50;
    private static final double TURRET_MIN_POWER = 0.04;

    // Negative = aim RIGHT, Positive = aim LEFT
    private static final double AIM_OFFSET_DEG_FAR = -5.0;
    private static final double AIM_OFFSET_DEG_CLOSE = -5.0; // was -3.0, now 2° more right

    // tx filtering (less lag, less twitch)
    private double txFiltered = 0.0;
    private static final double TX_FILTER_ALPHA = 0.45;
    private double lastTx = 0.0;
    private boolean hadTagLastLoop = false;

    // PD state
    private double prevAimError = 0.0;

    // Slew limiting to kill shake
    private double lastTurretPower = 0.0;
    private static final double TURRET_SLEW_PER_LOOP = 0.06; // max change per loop

    // ====== SHOOTER (pose-based distance → RPM) ======
    private static final double TICKS_PER_REV = 28.0;
    private static final double G = 9.80665;

    private static final double RPM_MIN = 600;
    private static final double RPM_MAX = 4500;

    // geometry
    private static final double WHEEL_DIAMETER_IN = 4.0;
    private static final double SHOOTER_HEIGHT_IN = 17.5;
    private static final double GOAL_ENTRY_IN = 43.0;
    private static final double SHOOTER_ANGLE_DEG = 60.0;

    private static final double SHOOTER_EFF = 0.55;

    private double rpmScale = 1.68;

    private double lastDistanceIn = 0.0;
    private double lastRpmCmd = 0.0;

    private int lastUsedFiducialId = -1;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

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
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("LT = aim, RT = shoot (only w/tag), RB = slow drive");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== LIVE RPM SCALE TUNING =====
            boolean dUp = gamepad1.dpad_up;
            boolean dDown = gamepad1.dpad_down;
            double step = gamepad1.dpad_left ? 0.10 : 0.02;

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
            double strafe =  gamepad1.left_stick_x;  // FIXED: no invert
            double turn   =  gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                drive *= 0.5;
                strafe *= 0.5;
                turn *= 0.5;
            }
            driveRobot(drive, strafe, turn);

            // ===== intake =====
            intake.setPower(gamepad1.a ? 1.0 : 0.0);

            // ===== kickers =====
            kick1.setPosition(gamepad1.y ? 0.7 : 0.1);
            kick2.setPosition(gamepad1.b ? 0.2 : 0.95);
            kick3.setPosition(gamepad1.x ? 0.45 : 1.0);

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
                            usedId = id;
                            hasTag24Now = true;

                            txNow = fid.getTargetXDegrees();
                            if (Math.abs(txNow) > 30.0) { hasTag24Now = false; break; }

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

                if (!hadTagLastLoop) {
                    txFiltered = txNow;
                    prevAimError = 0.0;
                } else {
                    txFiltered += TX_FILTER_ALPHA * (txNow - txFiltered);
                }
                lastTx = txFiltered;

                worldTargetAngle = wrapAngle(heading + wrapAngle(turretDeg));
                hasWorldTarget = true;
            } else {
                txFiltered = lastTx;
            }
            hadTagLastLoop = hasTag24Now;

            // ===== turret control =====
            boolean autoAim = gamepad1.left_trigger > 0.2;
            double turretPowerCmd = 0.0;
            String mode;

            if (autoAim) {
                if (hasTag24Now) {
                    double offset = (lastDistanceIn > 24.0) ? AIM_OFFSET_DEG_FAR : AIM_OFFSET_DEG_CLOSE;
                    double error = txFiltered + offset;
                    double absE = Math.abs(error);

                    double kp = (lastDistanceIn > 24.0) ? LIMELIGHT_KP_TURN_FAR : LIMELIGHT_KP_TURN_CLOSE;

                    if (absE <= LIMELIGHT_AIM_TOLERANCE) {
                        turretPowerCmd = 0.0;
                    } else {
                        double dErr = error - prevAimError;
                        prevAimError = error;

                        turretPowerCmd = (kp * error) - (LIMELIGHT_KD * dErr);
                        turretPowerCmd = clamp(turretPowerCmd, -LIMELIGHT_MAX_TURN, LIMELIGHT_MAX_TURN);

                        if (turretPowerCmd > 0) turretPowerCmd = Math.max(turretPowerCmd,  TURRET_MIN_POWER);
                        else                    turretPowerCmd = Math.min(turretPowerCmd, -TURRET_MIN_POWER);
                    }

                    mode = "LL tracking";
                } else if (hasWorldTarget) {
                    double turretWorldNow = wrapAngle(heading + wrapAngle(turretDeg));
                    double errorWorld = wrapAngle(worldTargetAngle - turretWorldNow);

                    if (Math.abs(errorWorld) <= ANGLE_DEADBAND_DEG) {
                        turretPowerCmd = 0.0;
                        mode = "World lock";
                    } else {
                        turretPowerCmd = K_AIM_WORLD * errorWorld;
                        turretPowerCmd = clamp(turretPowerCmd, -MAX_TURRET_POWER_WORLD, MAX_TURRET_POWER_WORLD);

                        if (turretPowerCmd > 0) turretPowerCmd = Math.max(turretPowerCmd,  WORLD_MIN_POWER);
                        else                    turretPowerCmd = Math.min(turretPowerCmd, -WORLD_MIN_POWER);

                        mode = "World lock";
                    }
                } else {
                    turretPowerCmd = 0.0;
                    mode = "Auto no lock";
                }
            } else {
                double bump = 0.6;
                if (gamepad1.left_bumper) turretPowerCmd = -bump;
                else if (gamepad1.right_bumper) turretPowerCmd = bump;
                else turretPowerCmd = 0.0;
                mode = "Manual";
            }

            // ===== Slew limit (kills shake) =====
            double lo = lastTurretPower - TURRET_SLEW_PER_LOOP;
            double hi = lastTurretPower + TURRET_SLEW_PER_LOOP;
            double turretPower = clamp(turretPowerCmd, lo, hi);
            lastTurretPower = turretPower;

            turret.setPower(turretPower);

            // ===== shooter =====
            boolean shoot = gamepad1.right_trigger > 0.2;
            if (shoot && hasTag24Now) {
                double rpm = computeRpmFromDistanceInches(lastDistanceIn, rpmScale);
                rpm = clamp(rpm, RPM_MIN, RPM_MAX);
                lastRpmCmd = rpm;
                outtake.setVelocity(rpmToTicksPerSecond(rpm, TICKS_PER_REV));
            } else {
                outtake.setVelocity(0.0);
            }

            telemetry.addData("Mode", mode);
            telemetry.addData("HasTag", hasTag24Now);
            telemetry.addData("txNow", txNow);
            telemetry.addData("txF", txFiltered);
            telemetry.addData("DistIn", lastDistanceIn);
            telemetry.addData("TurretP", turretPower);
            telemetry.addData("UsedId", lastUsedFiducialId);
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

        double v = Math.sqrt((G * x * x) / (2.0 * cos * cos * denom));

        double wheelDiamM = WHEEL_DIAMETER_IN * 0.0254;
        double wheelCircM = Math.PI * wheelDiamM;

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
