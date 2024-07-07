package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="LED Control", group="TeleOp")
public class LedTest extends OpMode {

    // Declare LED objects
    private DigitalChannel greenLED;
    private DigitalChannel redLED;

    @Override
    public void init() {
        // Initialize the hardware variables
        greenLED = hardwareMap.get(DigitalChannel.class, "LEDLeft");
        redLED = hardwareMap.get(DigitalChannel.class, "LEDLeftRed");

        // Set the direction of the LED channel
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
    }

    @Override
    public void loop() {
        // Check if button A is pressed on gamepad1
        if (gamepad1.a) {
            greenLED.setState(true);  // Turn on green color
            redLED.setState(false);   // Turn off red color
        }

        // Check if button B is pressed on gamepad1
        if (gamepad1.b) {
            greenLED.setState(false);  // Turn off green color
            redLED.setState(true);     // Turn on red color
        }
    }
}
