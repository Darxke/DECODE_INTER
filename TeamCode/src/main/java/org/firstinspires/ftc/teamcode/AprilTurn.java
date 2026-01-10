package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AprilTurn", group = "Test")
public class AprilTurn extends LinearOpMode {

    // Drivetrain motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // Shooter motors
    private DcMotorEx leftOut;
    private DcMotorEx rightOut;

    // Intake motor
    private DcMotor intake;
    private static final double INTAKE_POWER = 1.0;
    private boolean intakeOn = false;

    // Kicker servos
    private Servo kick1;
    private Servo kick2;

    private static final double KICK_REST_POS = 0.0;
    // Reduced kick position to 0.7 for kick1 and 0.8 for kick2
    private static final double KICK_FIRE_POS_1 = 0.4;  // Lowered position for kick1 when pressing A
    private static final double KICK_FIRE_POS_2 = 0.8;  // Reduced position for kick2

    // Delay for second kicker when pressing A (ms)
    private static final long KICK2_DELAY_MS = 500;

    // State for delayed A behavior
    private long aPressStartTime = 0;
    private boolean kick2TriggeredFromA = false;
    private boolean lastA = false;

    // Limelight 3A
    private Limelight3A limelight;

    // LIMELIGHT TURNING CONSTANTS
    static final double LIMELIGHT_KP_TURN = 0.02;
    private static final double LIMELIGHT_MAX_TURN = 0.4;
    private static final double LIMELIGHT_AIM_TOLERANCE = 1.5;

    // SHOOTER / PHYSICS CONSTANTS

    // CAMERA → for distance (floor -> camera lens)
    private static final double CAMERA_HEIGHT_IN = 11.0;
    // floor -> AprilTag CENTER
    private static final double APRILTAG_HEIGHT_IN = 29.75;
    // camera tilt upward (degrees)
    private static final double CAMERA_MOUNT_ANGLE_DEG = 5.0;

    // SHOOTER + HOOD → projectile to GOAL
    private static final double SHOOTER_HEIGHT_IN = 13.0;
    private static final double GOAL_HEIGHT_IN    = 45.0;
    private static final double SHOOTER_ANGLE_DEG = 60.0;

    // horizontal offset: shooter is 10" behind camera
    private static final double SHOOTER_FORWARD_OFFSET_IN = 10.0;

    // how far past the tag we aim
    private static final double EXTRA_SHOOT_DIST_IN = 8.0;

    // wheel and motor
    private static final double WHEEL_RADIUS_IN = 1.5;
    private static final double MAX_RPM = 6000.0;
    private static final double TICKS_PER_REV = 28.0;

    // gravity in inches/s^2
    private static final double G_INCHES = 386.08858;

    private static final double MIN_RPM = 0.0;

    // smoothing
    private static final double RPM_SMOOTH_ALPHA = 0.3;
    private static final double MAX_RPM_STEP     = 400.0;

    // stability / ready
    private double lastAimDist = Double.NaN;
    private int stableFrames = 0;

    // RPM smoothing
    private double lastRequiredRpm = Double.NaN;

    // Shooter enabled flag (B = start, X = stop)
    private boolean shooterEnabled = false;

    // ===== CACHED APRILTAG DATA =====
    private double lastTx = 0.0;
    private double lastTy = 0.0;
    private boolean haveTargetCached = false;
    private int lostTargetFrames = 0;
    private static final int MAX_LOST_FRAMES = 10; // how many loops to trust last reading

    @Override
    public void runOpMode() throws InterruptedException {
        // DRIVETRAIN HARDWARE
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

        // SHOOTER HARDWARE
        leftOut  = hardwareMap.get(DcMotorEx.class, "leftOut");
        rightOut = hardwareMap.get(DcMotorEx.class, "rightOut");

        leftOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftOut.setDirection(DcMotorSimple.Direction.FORWARD);
        rightOut.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // INTAKE HARDWARE
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // KICKER SERVOS
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");

        kick1.setPosition(KICK_REST_POS);
        kick2.setPosition(KICK_REST_POS);

        // LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("AprilTurn init complete");
        telemetry.addLine("LB = auto-face AprilTag (with cached fallback)");
        telemetry.addLine("B = start shooter, X = stop shooter");
        telemetry.addLine("A = kick1 up; after 500 ms, kick2 up (non-blocking)");
        telemetry.addLine("Y = kick2 up while held (overrides A-delay)");
        telemetry.addLine("RB = intake on (affects RPM scale)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;

            boolean aPressed  = gamepad1.a;
            boolean yPressed  = gamepad1.y;
            boolean rbPressed = gamepad1.right_bumper;
            boolean autoAim   = gamepad1.left_bumper;

            long nowMs = System.currentTimeMillis();

            // ===== GET LIMELIGHT RESULT & UPDATE CACHE =====
            LLResult result = limelight.getLatestResult();
            boolean hasNow = result != null && result.isValid();

            if (hasNow) {
                lastTx = result.getTx();
                lastTy = result.getTy();
                haveTargetCached = true;
                lostTargetFrames = 0;
            } else if (haveTargetCached && lostTargetFrames < MAX_LOST_FRAMES) {
                lostTargetFrames++;
            } else {
                haveTargetCached = false;
            }

            // Shooter toggles
            if (gamepad1.b) {
                shooterEnabled = true;
            }
            if (gamepad1.x) {
                shooterEnabled = false;
                stopShooters();
            }

            // Intake control + state
            if (rbPressed) {
                intakeOn = true;
                intake.setPower(INTAKE_POWER);
            } else {
                intakeOn = false;
                intake.setPower(0.0);
            }

            // ===== KICKER LOGIC WITH DELAYED SECOND KICKER =====

            // Rising edge for A -> record press time and reset triggered flag
            if (aPressed && !lastA) {
                aPressStartTime = nowMs;
                kick2TriggeredFromA = false;
            }
            lastA = aPressed;

            // Kick1 is simple: A held = up, else down
            if (aPressed) {
                kick1.setPosition(KICK_FIRE_POS_1);  // Lowered position for kick1
            } else {
                kick1.setPosition(KICK_REST_POS);
            }

            // Kick2 default position
            double kick2Pos = KICK_REST_POS;

            // Y direct override: if Y held, kick2 is up no matter what
            if (yPressed) {
                kick2Pos = KICK_FIRE_POS_2;  // Lowered position for kick2 when pressing Y
            } else {
                // No Y, so A-delay behavior is allowed to control kick2
                if (aPressed) {
                    // Check if enough time has passed since A press
                    long elapsed = nowMs - aPressStartTime;
                    if (!kick2TriggeredFromA && elapsed >= KICK2_DELAY_MS) {
                        kick2TriggeredFromA = true;
                    }
                    if (kick2TriggeredFromA) {
                        kick2Pos = KICK_FIRE_POS_2;  // Reduced position for kick2
                    }
                } else {
                    // A not held anymore: reset the delayed state
                    kick2TriggeredFromA = false;
                    kick2Pos = KICK_REST_POS;
                }
            }

            kick2.setPosition(kick2Pos);

            // Drive with or without auto-aim (using cached target if needed)
            if (autoAim) {
                double limelightTurn = getLimelightTurnCommand(haveTargetCached, lastTx);
                driveRobot(drive, strafe, limelightTurn);
            } else {
                driveRobot(drive, strafe, turn);
            }

            // Shooter RPM control, using cached ty when needed
            if (shooterEnabled) {
                updateShooterVelocity(haveTargetCached, lastTy);
            }

            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("Intake On", intakeOn);
            telemetry.addData("Tag Cached", haveTargetCached);
            telemetry.addData("Lost Target Frames", lostTargetFrames);
            telemetry.addData("Last tx", lastTx);
            telemetry.addData("Last ty", lastTy);
            telemetry.addData("Kick1 Pos", kick1.getPosition());
            telemetry.addData("Kick2 Pos", kick2.getPosition());
            telemetry.addData("Kick2 From A", kick2TriggeredFromA);
            telemetry.update();
        }

        stopShooters();
        intake.setPower(0.0);
    }

    // LIMELIGHT TURN CONTROL (using cached tx)
    private double getLimelightTurnCommand(boolean hasTarget, double tx) {
        if (!hasTarget) {
            telemetry.addLine("No usable AprilTag (no cache or expired)");
            return 0.0;
        }

        double turnCmd = LIMELIGHT_KP_TURN * tx;

        if (turnCmd > LIMELIGHT_MAX_TURN) {
            turnCmd = LIMELIGHT_MAX_TURN;
        } else if (turnCmd < -LIMELIGHT_MAX_TURN) {
            turnCmd = -LIMELIGHT_MAX_TURN;
        }

        boolean aimLocked = Math.abs(tx) < LIMELIGHT_AIM_TOLERANCE;

        if (aimLocked) {
            telemetry.addLine("AIM LOCKED (cached or live)");
            return 0.0;
        } else {
            telemetry.addData("tx", tx);
            telemetry.addData("turnCmd", turnCmd);
            return turnCmd;
        }
    }

    // SHOOTER VELOCITY LOGIC (using cached ty)
    private void updateShooterVelocity(boolean hasTarget, double ty) {
        double camDistInches = 0.0;
        double shooterDistInches = 0.0;
        double aimDistInches = Double.NaN;

        double requiredVelInPerSec = Double.NaN;
        double requiredRpmRaw = Double.NaN;
        double requiredRpm = Double.NaN;
        double targetTicksPerSec = 0.0;
        double rpmScale = 1.0;
        boolean shooterReady = false;

        if (hasTarget) {
            camDistInches = calculateCameraDistanceInches(ty);

            shooterDistInches = camDistInches + 10.0;  // Offset to shooter
            aimDistInches = shooterDistInches + EXTRA_SHOOT_DIST_IN;

            requiredVelInPerSec = getRequiredExitVelocity(aimDistInches);

            if (!Double.isNaN(requiredVelInPerSec)) {
                requiredRpmRaw = velocityToWheelRpm(requiredVelInPerSec, WHEEL_RADIUS_IN);

                // Adjusted scaling for close distances
                if (!Double.isNaN(aimDistInches) && aimDistInches <= 80.0) {
                    rpmScale = 1.4;  // Increased scale for close distances
                    if (intakeOn) {
                        rpmScale += 0.2;  // bonus when intake on & close
                    }
                } else {
                    rpmScale = 1.3;  // Scaling for farther distances
                    if (intakeOn) {
                        rpmScale += 0.1;  // bonus when intake on & far
                    }
                }


                double scaledRpm = requiredRpmRaw * rpmScale;

                if (scaledRpm < 0) scaledRpm = 0;
                if (scaledRpm > MAX_RPM) scaledRpm = MAX_RPM;

                if (Double.isNaN(lastRequiredRpm)) {
                    requiredRpm = scaledRpm;
                } else {
                    double blended = lastRequiredRpm
                            + RPM_SMOOTH_ALPHA * (scaledRpm - lastRequiredRpm);

                    double delta = blended - lastRequiredRpm;
                    if (delta > MAX_RPM_STEP) delta = MAX_RPM_STEP;
                    if (delta < -MAX_RPM_STEP) delta = -MAX_RPM_STEP;

                    requiredRpm = lastRequiredRpm + delta;
                }
                lastRequiredRpm = requiredRpm;

                if (requiredRpm > 0 && requiredRpm < MIN_RPM) {
                    requiredRpm = MIN_RPM;
                }

                if (requiredRpm < 0) requiredRpm = 0;
                if (requiredRpm > MAX_RPM) requiredRpm = MAX_RPM;

                targetTicksPerSec = rpmToTicksPerSecond(requiredRpm, TICKS_PER_REV);

                leftOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftOut.setVelocity(targetTicksPerSec);
                rightOut.setVelocity(targetTicksPerSec);

                if (!Double.isNaN(aimDistInches)) {
                    if (!Double.isNaN(lastAimDist)
                            && Math.abs(aimDistInches - lastAimDist) < 2.0) {
                        stableFrames++;
                    } else {
                        stableFrames = 0;
                    }
                    lastAimDist = aimDistInches;

                    if (stableFrames >= 5 && hasTarget && requiredRpm > 0) {
                        shooterReady = true;
                    } else {
                        shooterReady = false;
                    }
                } else {
                    stableFrames = 0;
                    shooterReady = false;
                }
            } else {
                stopShooters();
                targetTicksPerSec = 0;
                stableFrames = 0;
                shooterReady = false;
                lastRequiredRpm = Double.NaN;
            }
        } else {
            stopShooters();
            stableFrames = 0;
            shooterReady = false;
            lastRequiredRpm = Double.NaN;
        }

        telemetry.addData("Has Target (live/cached)", hasTarget);
        telemetry.addData("ty (cached/live)", ty);
        telemetry.addData("Cam Dist to Tag (in)", camDistInches);
        telemetry.addData("Shoot Dist to Tag (in)", shooterDistInches);
        telemetry.addData("Aim Dist (tag+offset, in)", aimDistInches);
        telemetry.addData("Req v (in/s)", requiredVelInPerSec);
        telemetry.addData("Raw RPM (physics)", requiredRpmRaw);
        telemetry.addData("RPM Scale", rpmScale);
        telemetry.addData("Req RPM (smoothed)", requiredRpm);
        telemetry.addData("Target Vel (ticks/s)", targetTicksPerSec);
        telemetry.addData("Stable Frames", stableFrames);
        telemetry.addData("Shooter Ready", shooterReady);
    }

    private void stopShooters() {
        leftOut.setVelocity(0);
        rightOut.setVelocity(0);
    }

    private double calculateCameraDistanceInches(double tyDeg) {
        double angleToTargetDeg = CAMERA_MOUNT_ANGLE_DEG + tyDeg;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);
        return (APRILTAG_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleToTargetRad);
    }

    private double getRequiredExitVelocity(double dInches) {
        double thetaRad = Math.toRadians(SHOOTER_ANGLE_DEG);
        double deltaH = GOAL_HEIGHT_IN - SHOOTER_HEIGHT_IN;

        double numerator = G_INCHES * dInches * dInches;
        double denomInside = dInches * Math.tan(thetaRad) - deltaH;

        if (denomInside <= 0) {
            return Double.NaN;
        }

        double denominator = 2.0
                * Math.cos(thetaRad) * Math.cos(thetaRad)
                * denomInside;

        return Math.sqrt(numerator / denominator);
    }

    private double velocityToWheelRpm(double vInchesPerSec, double wheelRadiusInches) {
        if (wheelRadiusInches <= 0) return Double.NaN;
        double omegaRadPerSec = vInchesPerSec / wheelRadiusInches;
        return omegaRadPerSec * 60.0 / (2.0 * Math.PI);
    }

    private double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }

    // MECANUM DRIVE
    private void driveRobot(double drive, double strafe, double turn) {
        double lf = drive + strafe + turn;
        double rf = drive - strafe - turn;
        double lb = drive - strafe + turn;
        double rb = drive + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        lf /= max;
        rf /= max;
        lb /= max;
        rb /= max;

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }
}
