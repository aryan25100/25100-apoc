package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.wrappers.JServo;

@Autonomous
public class ServoTestFour extends LinearOpMode {
    public JServo v4Bar, transferFlap, leftPitch, rightPitch, pivot, roll, fingerLeft, fingerRight, droneServo;
    double pivotpos,pitchpos;
    @Override
    public void runOpMode() throws InterruptedException {
        pivot = new JServo(hardwareMap.get(Servo.class, "pivotServo"));
        leftPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoLeft"));
        rightPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoRight"));

        pivotpos=0.5;
        pitchpos=0.5;
        waitForStart();

        while (!isStopRequested()){

            if (gamepad1.dpad_left){
                pivotpos=pivotpos-0.01;
                pivot.setPosition(pivotpos);
                sleep(100);
            }
            if (gamepad1.dpad_right){
                pivotpos=pivotpos+0.01;
                pivot.setPosition(pivotpos);
                sleep(100);
            }
            if (gamepad1.y){
                pitchpos=pitchpos+0.01;
                rightPitch.setPosition(pitchpos);
                leftPitch.setPosition(pitchpos);
                sleep(100);
            }
            if (gamepad1.a){
                pitchpos=pitchpos-0.01;
                rightPitch.setPosition(pitchpos);
                leftPitch.setPosition(pitchpos);
                sleep(100);
            }

            telemetry.addData("pitch",pitchpos);
            telemetry.addData("pivbot", pivotpos);
            telemetry.update();
        }
    }
}
