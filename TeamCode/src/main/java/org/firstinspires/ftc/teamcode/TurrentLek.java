package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "TurrentManual_Final_V2", group = "Production")
public class TurrentLek extends LinearOpMode {

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake;
    private DcMotorEx outtakeL, outtakeR, turret;
    private Servo kick1, kick2, kick3, kick4;
    private RevBlinkinLedDriver led;
    private List<LynxModule> allHubs;

    private static final double RPM_CLOSE = 2571;
    private static final double RPM_FAR   = 3268;
    private static final double TICKS_PER_REV = 28.0;
    private static final double VELOCITY_TOLERANCE = 45.0;

    private double velocityAdjustment = 0;
    private final double ADJUST_STEP = 50.0;

    private boolean farMode = false;
    private static final double TURRET_POWER = 0.30;

    // Kicker positions
    private static final double K1_REST = 0.1, K1_FIRE = 0.7;
    private static final double K2_REST = 0.1, K2_FIRE = 0.95;
    private static final double K3_REST = 1.0, K3_FIRE = 0.45;
    private static final double K4_REST = 1.0, K4_FIRE = 0.40;
    private static final int FIRE_MS = 500;

    // Edge detection
    private boolean lastX2 = false, lastB2 = false, lastY2 = false, lastA2 = false;
    private boolean lastDpadLeft2 = false, lastDpadRight2 = false;

    @Override
    public void runOpMode() throws InterruptedException {

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Hardware Mapping
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        DcMotorEx[] drive = { leftFront, rightFront, leftBack, rightBack };
        for (DcMotorEx m : drive) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");
        outtakeR.setDirection(DcMotorSimple.Direction.REVERSE); // Ensure opposing rotation for dual outtake

        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick3 = hardwareMap.get(Servo.class, "kick3");
        kick4 = hardwareMap.get(Servo.class, "kick4");
        setAllKickersRest();

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            // 1. DRIVE (Gamepad 1)
            double y = -gamepad1.left_stick_y, x = gamepad1.left_stick_x, t = gamepad1.right_stick_x;
            double lf = y+x+t, rf = y-x-t, lb = y-x+t, rb = y+x-t;
            double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
            leftFront.setPower(lf/max); rightFront.setPower(rf/max);
            leftBack.setPower(lb/max); rightBack.setPower(rb/max);

            // 2. INTAKE (Gamepad 2 Start/Back)
            if (gamepad2.right_bumper) intake.setPower(1.0);
            else if (gamepad2.left_bumper) intake.setPower(-1.0);
            else intake.setPower(0);

            // 3. TURRET (Gamepad 2 Bumpers)
            if (gamepad1.left_bumper) turret.setPower(-TURRET_POWER);
            else if (gamepad1.right_bumper) turret.setPower(TURRET_POWER);
            else turret.setPower(0);

            // 4. MODE SELECTION (D-pad Up/Down)
            if (gamepad2.dpad_up) {
                farMode = true;
                velocityAdjustment = 0;
            } else if (gamepad2.dpad_down) {
                farMode = false;
                velocityAdjustment = 0;
            }

            // 5. VELOCITY FINE TUNE (D-pad Left/Right)
            if (gamepad2.dpad_left && !lastDpadLeft2)   velocityAdjustment -= ADJUST_STEP;
            if (gamepad2.dpad_right && !lastDpadRight2) velocityAdjustment += ADJUST_STEP;
            lastDpadLeft2 = gamepad2.dpad_left;
            lastDpadRight2 = gamepad2.dpad_right;

            // 6. SHOOTER OUTPUT
            boolean shooterOn = gamepad2.right_trigger > 0.2;
            double baseRpm = farMode ? RPM_FAR : RPM_CLOSE;
            double targetRpm = baseRpm + velocityAdjustment;
            double targetVel = (targetRpm * TICKS_PER_REV) / 60.0;

            if (shooterOn) {
                outtakeL.setVelocity(targetVel); outtakeR.setVelocity(targetVel);
                double avgVel = (outtakeL.getVelocity() + outtakeR.getVelocity()) / 2.0;
                led.setPattern(Math.abs(avgVel - targetVel) < VELOCITY_TOLERANCE ?
                        RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE : RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
            } else {
                outtakeL.setVelocity(0); outtakeR.setVelocity(0);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

            // 7. KICKERS (Face Buttons)
            if (gamepad2.x && !lastX2) fireKicker(1);
            if (gamepad2.b && !lastB2) fireKicker(2);
            if (gamepad2.y && !lastY2) fireKicker(3);
            if (gamepad2.a && !lastA2) fireKicker(4);

            lastX2 = gamepad2.x; lastB2 = gamepad2.b; lastY2 = gamepad2.y; lastA2 = gamepad2.a;

            telemetry.addData("Shooter Mode", farMode ? "FAR" : "CLOSE");
            telemetry.addData("Target RPM", "%.0f (Base: %.0f, Adj: %.0f)", targetRpm, baseRpm, velocityAdjustment);
            telemetry.addData("Current RPM", (outtakeL.getVelocity() * 60.0) / TICKS_PER_REV);
            telemetry.update();
        }
    }

    private void fireKicker(int k) {
        if (k == 1) { kick1.setPosition(K1_FIRE); sleep(FIRE_MS); kick1.setPosition(K1_REST); }
        else if (k == 2) { kick2.setPosition(K2_FIRE); sleep(FIRE_MS); kick2.setPosition(K2_REST); }
        else if (k == 3) { kick3.setPosition(K3_FIRE); sleep(FIRE_MS); kick3.setPosition(K3_REST); }
        else if (k == 4) { kick4.setPosition(K4_FIRE); sleep(FIRE_MS); kick4.setPosition(K4_REST); }
    }

    private void setAllKickersRest() {
        kick1.setPosition(K1_REST); kick2.setPosition(K2_REST);
        kick3.setPosition(K3_REST); kick4.setPosition(K4_REST);
    }
}