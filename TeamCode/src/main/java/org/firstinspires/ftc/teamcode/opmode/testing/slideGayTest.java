package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class slideGayTest extends LinearOpMode {

    private DcMotor slideLeft;
    private DcMotor slideRight;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Check if the key for moving the slides up is pressed (e.g., gamepad1.a)
            if (gamepad1.a) {
                slideLeft.setPower(1);
                slideRight.setPower(1);
            }
            // Check if the key for moving the slides down is pressed (e.g., gamepad1.b)
            else if (gamepad1.b) {
                slideLeft.setPower(-1);
                slideRight.setPower(-1);
            }
            // Otherwise, stop the slides
            else {
                slideLeft.setPower(0);
                slideRight.setPower(0);
            }

            // Send telemetry message to signify robot running
            telemetry.addData("Slide1 Power", slideLeft.getPower());
            telemetry.addData("Slide2 Power", slideRight.getPower());
            telemetry.update();
        }
    }
}
