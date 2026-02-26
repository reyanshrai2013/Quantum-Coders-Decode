package org.firstinspires.ftc.teamcode.pedroPathing.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * RGB INDICATOR TUNER — uses FTC Dashboard
 * -----------------------------------------
 * 1. Connect to Control Hub WiFi
 * 2. Go to 192.168.43.1:8080/dash
 * 3. Run this OpMode
 * 4. Find the "RGBTunerOpMode" config panel on the right side
 * 5. Adjust "position" (0.0 to 1.0) live — light updates instantly!
 * 6. Copy the final value into your main code.
 */
@Config
@Disabled
@TeleOp(name = "RGB Indicator Tuner", group = "Tuning")
public class RGBTunerOpMode extends OpMode {

    // Shows up as a live-editable field in the FTC Dashboard config panel
    public static double position = 0.0;

    private Servo light;

    @Override
    public void init() {
        // TODO: replace "light" with your actual hardware map name
        light = hardwareMap.get(Servo.class, "light");

        telemetry.addLine("RGB Tuner ready!");
        telemetry.addLine("Adjust 'position' in the Dashboard config panel (0.0 - 1.0).");
        telemetry.update();
    }

    @Override
    public void loop() {
        double pos = Math.max(0.0, Math.min(1.0, position));

        light.setPosition(pos);

        telemetry.addData("position", pos);
        telemetry.update();
    }
}