package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Diagnostic: Encoder Check", group = "Test")
public class EncoderDiagnostic extends OpMode {

    private DcMotorEx turretMotor;

    @Override
    public void init() {
        // Use the same name as your other code
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        // Reset the encoder to zero at the start
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Initialized. Press Start to test.");
    }

    @Override
    public void loop() {
        // Use the Left Joystick to move the turret manually
        double stickPower = -gamepad1.left_stick_x * 0.3; // Cap at 30% power
        turretMotor.setPower(stickPower);

        // Get the current position from the encoder
        int currentPosition = turretMotor.getCurrentPosition();

        // TELEMETRY - This is what you watch on the screen
        telemetry.addData("TESTING", "Move Left Stick X to spin motor");
        telemetry.addLine("----------------------------------");
        telemetry.addData("Motor Power", "%.2f", stickPower);
        telemetry.addData("ENCODER TICKS", currentPosition);

        // Logical check for you
        if (Math.abs(stickPower) > 0.05 && currentPosition == 0) {
            telemetry.addLine("!!! WARNING: Motor is moving but Ticks are 0 !!!");
            telemetry.addLine("Check your encoder wire connections.");
        } else if (Math.abs(stickPower) > 0.05) {
            telemetry.addLine("Encoder appears to be WORKING.");
        }

        telemetry.update();
    }
}