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

@TeleOp(name = "TeleOp_SplitControllers_DualOuttake", group = "Production")
public class TurrentLek extends LinearOpMode {

    // Drive
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // Mechanisms
    private DcMotor intake;
    private DcMotorEx outtakeL, outtakeR;  // dual shooter motors
    private DcMotorEx turret;

    // Kickers
    private Servo kick1, kick2, kick3;

    // LED
    private RevBlinkinLedDriver led;

    // Bulk caching
    private List<LynxModule> allHubs;

    // Shooter settings
    private static final double RPM_CLOSE = 2571;
    private static final double RPM_FAR   = 3268;

    // Encoder
    private static final double TICKS_PER_REV = 28.0;

    private static final double VELOCITY_TOLERANCE = 45.0;
    private boolean farMode = false;

    // Turret
    private static final double TURRET_POWER = 0.5;

    // Kicker positions
    private static final double K1_REST = 0.1, K1_FIRE = 0.7;
    private static final double K2_REST = 0.1, K2_FIRE = 0.95;
    private static final double K3_REST = 1.0, K3_FIRE = 0.45;
    private static final int FIRE_MS = 500;

    // Edge detection for kickers (gamepad2)
    private boolean lastX2 = false, lastB2 = false, lastY2 = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- BULK READ ----------------
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ---------------- DRIVE MOTORS ----------------
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        DcMotorEx[] drive = { leftFront, rightFront, leftBack, rightBack };
        for (DcMotorEx m : drive) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- INTAKE ----------------
        intake = hardwareMap.get(DcMotor.class, "intake");

        // ---------------- SHOOTER (DUAL OUTTAKE) ----------------
        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        // Reverse one motor so both spin correctly
        outtakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeR.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional: PIDF for precise velocity control
        outtakeL.setVelocityPIDFCoefficients(50, 0, 5, 12);
        outtakeR.setVelocityPIDFCoefficients(50, 0, 5, 12);

        // ---------------- TURRET ----------------
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------------- KICKERS ----------------
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick3 = hardwareMap.get(Servo.class, "kick3");
        setAllKickersRest();

        // ---------------- LED ----------------
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            // ---------------- GAMEPAD 1: DRIVE ----------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double t = gamepad1.right_stick_x;

            double lf = y + x + t;
            double rf = y - x - t;
            double lb = y - x + t;
            double rb = y + x - t;

            double max = Math.max(1.0,
                    Math.max(Math.abs(lf),
                            Math.max(Math.abs(rf),
                                    Math.max(Math.abs(lb), Math.abs(rb)))));

            leftFront.setPower(lf / max);
            rightFront.setPower(rf / max);
            leftBack.setPower(lb / max);
            rightBack.setPower(rb / max);

            // ---------------- GAMEPAD 1: INTAKE ----------------
            intake.setPower(gamepad1.a ? 1.0 : 0.0);

            // ---------------- GAMEPAD 2: TURRET ----------------
            if (gamepad2.left_bumper) {
                turret.setPower(-TURRET_POWER);
            } else if (gamepad2.right_bumper) {
                turret.setPower(TURRET_POWER);
            } else {
                turret.setPower(0);
            }

            // ---------------- GAMEPAD 2: SHOOTER MODE ----------------
            if (gamepad2.dpad_up) farMode = true;
            if (gamepad2.dpad_down) farMode = false;

            // ---------------- GAMEPAD 2: SHOOTER ----------------
            boolean shooterOn = gamepad2.right_trigger > 0.2;
            double targetRpm = farMode ? RPM_FAR : RPM_CLOSE;
            double targetVel = rpmToTicksPerSecond(targetRpm);

            if (shooterOn) {
                outtakeL.setVelocity(targetVel);
                outtakeR.setVelocity(targetVel);

                double avgVel = (outtakeL.getVelocity() + outtakeR.getVelocity()) / 2.0;

                if (Math.abs(avgVel - targetVel) < VELOCITY_TOLERANCE) {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                } else {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
                }
            } else {
                outtakeL.setVelocity(0);
                outtakeR.setVelocity(0);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

            // ---------------- GAMEPAD 2: KICKERS ----------------
            boolean x2 = gamepad2.x;
            boolean b2 = gamepad2.b;
            boolean y2 = gamepad2.y;

            if (x2 && !lastX2) fireKicker(1);
            if (b2 && !lastB2) fireKicker(2);
            if (y2 && !lastY2) fireKicker(3);

            lastX2 = x2;
            lastB2 = b2;
            lastY2 = y2;

            // ---------------- TELEMETRY ----------------
            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Mode", farMode ? "FAR" : "CLOSE");
            telemetry.addData("Outtake L Vel", outtakeL.getVelocity());
            telemetry.addData("Outtake R Vel", outtakeR.getVelocity());
            telemetry.update();
        }
    }

    // ---------------- HELPERS ----------------
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private void fireKicker(int k) {
        switch (k) {
            case 1:
                kick1.setPosition(K1_FIRE);
                sleep(FIRE_MS);
                kick1.setPosition(K1_REST);
                break;
            case 2:
                kick2.setPosition(K2_FIRE);
                sleep(FIRE_MS);
                kick2.setPosition(K2_REST);
                break;
            case 3:
                kick3.setPosition(K3_FIRE);
                sleep(FIRE_MS);
                kick3.setPosition(K3_REST);
                break;
        }
    }

    private void setAllKickersRest() {
        kick1.setPosition(K1_REST);
        kick2.setPosition(K2_REST);
        kick3.setPosition(K3_REST);
    }
}
