package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ServoTest extends LinearOpMode {
    private Servo droneheight,dronelaunch;
    private double pos=0.5;
    private double pos1=0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested() && !isStarted()) {
            droneheight = hardwareMap.get(Servo.class, "droneHeightServo");
            dronelaunch = hardwareMap.get(Servo.class, "droneLaunchServo");
            pos= 0.5;
            pos1 = 0.5;
            droneheight.setPosition(pos);
        }

        while (!isStopRequested()){

            if (gamepad1.dpad_left){
                pos=pos-0.01;
                droneheight.setPosition(pos);
                sleep(100);
            }
            if (gamepad1.dpad_right){
                pos=pos+0.01;
                droneheight.setPosition(pos);
                sleep(100);
            }
            if (gamepad2.dpad_up){
                pos1 = 1;
                dronelaunch.setPosition(pos1);
                sleep(100);
            }
            if (gamepad2.dpad_down){
                pos1 = 0;
                dronelaunch.setPosition(pos1);
                sleep(100);
            }
            droneheight.setPosition(pos);
            dronelaunch.setPosition(pos1);
            //launch at 1 rest at 0 for shooter


            telemetry.addData("droneservoPos",pos);
            telemetry.update();
        }
    }
}
