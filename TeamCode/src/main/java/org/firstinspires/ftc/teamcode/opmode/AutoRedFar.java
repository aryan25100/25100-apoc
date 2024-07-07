package org.firstinspires.ftc.teamcode.opmode;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToDropPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToSpikeDrop;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToWaitPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToDropPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.PivotToSpikeDrop;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToWaitPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.slideToRow;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.followPath;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.flapDownStationary;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.outtakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.stopIntake;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.v4BarToHeight;
import org.firstinspires.ftc.teamcode.common.pathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.*;
import java.util.function.BooleanSupplier;


@Autonomous
public class AutoRedFar extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private double currentTime=0;
    private double lastTime = 0.0;
    private double loopTime=0;
    private Path toSpikeLeft;
    private Path toSpikeMiddle;
    private Path toSpikeRight;
    private Path toStackMiddle;
    private Path toStackLeftFromBB;
    private Path toStrafeAtLeftStack;
    private Path toBackboard;
    private Path toBackboardRight;
    private Path toBackboardMiddle;
    private Path toBackboardLeft;
    private Path toPark;
    private Path toBackboardfromstack;
    private Path toBackboardSecondTime;
    private Path TostackSecondTime;
    private Path TostrafeSecondTime;


    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() >26;
    private BooleanSupplier preload = () -> robot.preloadDetectionPipeline.getPreloadedZone() == CenterstageConstants.PRELOAD;
    private BooleanSupplier extend = () -> robot.poseUpdater.getPose().getX()>20;
    private int zone;
    double bbDropY=-36;
    double bbDropX=45;
    long wait = 0;
    @Override
    public void initialize() {


        telemetry.setMsTransmissionInterval(50);


        toSpikeMiddle = new Path(new BezierLine(new Point(-39, -56,Point.CARTESIAN), new Point(-55, -21.5,Point.CARTESIAN)));
        toSpikeMiddle.setLinearHeadingInterpolation(Math.toRadians(90),0);
        toSpikeLeft = new Path(new BezierLine(new Point(-39, -56,Point.CARTESIAN), new Point(-58, -17,Point.CARTESIAN)));
        toSpikeLeft.setConstantHeadingInterpolation(Math.toRadians(-44));
        toSpikeRight = new Path(new BezierLine(new Point(-39, -56,Point.CARTESIAN), new Point(-41.4, -32.6,Point.CARTESIAN)));
        toSpikeRight.setConstantHeadingInterpolation(Math.toRadians(0));

        toStackMiddle = new Path(new BezierLine(new Point(-55,-21.5,Point.CARTESIAN),new Point(-60, -21.5, Point.CARTESIAN)));
        toStackMiddle.setConstantHeadingInterpolation(Math.toRadians(0));

        toStackLeftFromBB = new Path(new BezierCurve((new Point(43,-35,Point.CARTESIAN)),(new Point(30,2,Point.CARTESIAN)),(new Point(-27,4,Point.CARTESIAN)),(new Point(-59, -15,Point.CARTESIAN))));
        toStackLeftFromBB.setConstantHeadingInterpolation(0);
//        toStackLeftFromBB.setReversed(true);

        toStrafeAtLeftStack = new Path(new BezierLine((new Point(-59, -15, Point.CARTESIAN)),(new Point(-60,-25,Point.CARTESIAN))));
        toStrafeAtLeftStack.setConstantHeadingInterpolation(0);

        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        CenterstageConstants.ALLIANCE = Location.RED;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);

        robot.follower.setStartingPose(new Pose(-39,-58,Math.toRadians(90)));
        while (opModeInInit()) {

            zone=robot.propDetectionPipeline.detectZone();

            if (zone==1){
                bbDropX=44.5;
                bbDropY=-23;
                wait = 1000;
            }
            else if (zone==2) {
                bbDropX=44.5;
                bbDropY=-25;
                wait = 250;
            }
            else{
                bbDropX=44.5;
                bbDropY=-28;
                wait = 1000;
            }

            toBackboard = new Path(new BezierCurve(new Point(-57, -14,Point.CARTESIAN), new Point(-27,0,Point.CARTESIAN), new Point(30,0,Point.CARTESIAN),new Point(43,-14 ,Point.CARTESIAN),new Point(bbDropX, bbDropY,Point.CARTESIAN)));
            toBackboardfromstack = new Path(new BezierCurve(new Point(-62 , -14,Point.CARTESIAN), new Point(-27,0,Point.CARTESIAN), new Point(30,0,Point.CARTESIAN),new Point(43,-14 ,Point.CARTESIAN),new Point(44.2 ,-28.5 , Point.CARTESIAN)));
            //tp stack again
            toBackboardSecondTime =  new Path(new BezierCurve(new Point(-57, -14,Point.CARTESIAN), new Point(-27,0,Point.CARTESIAN), new Point(30,-3,Point.CARTESIAN),new Point(43,-14 ,Point.CARTESIAN),new Point(44.5 ,-28 , Point.CARTESIAN)));
            toBackboardSecondTime.setConstantHeadingInterpolation(0);
            toBackboard.setConstantHeadingInterpolation(Math.toRadians(0));
            toBackboardfromstack.setConstantHeadingInterpolation(Math.toRadians(-2 ));
            TostackSecondTime = new Path(new BezierCurve((new Point(43,-35,Point.CARTESIAN)),(new Point(30,0,Point.CARTESIAN)),(new Point(-27,0,Point.CARTESIAN)),(new Point(-60.5 , -13,Point.CARTESIAN))));
            TostackSecondTime.setConstantHeadingInterpolation(0);

            TostrafeSecondTime = new Path(new BezierLine((new Point(-58, -5, Point.CARTESIAN)),(new Point(-60,-15,Point.CARTESIAN))));
            TostrafeSecondTime.setConstantHeadingInterpolation(0);
            CommandScheduler.getInstance().reset();


            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new followPath(CenterstageConstants.getPath(zone, toSpikeLeft,toSpikeMiddle,toSpikeRight)),
                                    new SequentialCommandGroup(
                                            new pitchToTransferPosition(),
                                            new pivotToTransferPosition(),
                                            new WaitCommand(100),
                                            new grabRightPixel(),
                                            new releaseLeftPixel(),
                                            new WaitCommand(wait),
                                            new pitchToSpikeDrop(),
                                            new WaitCommand(100),
                                            new PivotToSpikeDrop(),
                                            new WaitCommand(300)
                                    ),
                                    new WaitUntilCommand(busy)
                            ),
                            new WaitCommand(200),
                            new releaseRightPixel(),
                            new WaitCommand(100),
                            //below is new
                            new v4BarToHeight(5),
                            new intakeCommand(), // make intake
                            new ParallelCommandGroup(
                                    new followPath(toStackMiddle),
                                    new ParallelRaceGroup(
                                            new WaitCommand(1500),
                                            new WaitUntilCommand(pixels)
                                    )
                            ),
                            new WaitCommand(300),
                            new stopIntake(),
                            new outtakeCommand(), //make outtake
                            new WaitCommand(250),
                            new stopIntake(),
                            //above is new
//                            new pivotToTransferPosition(),
//                            new pitchToTransferPosition(),
//                            new WaitCommand(400),
//                            new grabLeftPixel(),
//                            //below is new
//                            new grabRightPixel()
                            new ParallelCommandGroup(
                                    new followPath(toBackboard),
                                    new SequentialCommandGroup(
                                            new pivotToTransferPosition(),
                                            new pitchToTransferPosition(),
                                            new WaitCommand(400),
                                            new grabLeftPixel(),
                                            new grabRightPixel(),
                                            new WaitUntilCommand(extend),
                                            new slideToRow(2),
                                            new pitchToDropPosition(),
                                            new WaitCommand(100),
                                            new pivotToDropPosition()

                                            //below is new
                                    ),
                                    new WaitUntilCommand(busy)
                            ),
                            new WaitCommand(150),
                            new releaseLeftPixel(),
                            new releaseRightPixel(),
                            new WaitCommand(200),
                            new slideToRow(1),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),

                            new followPath(toStackLeftFromBB),
                            new flapDownStationary(),
                            new WaitUntilCommand(busy),
                            new followPath(toStrafeAtLeftStack),
                            new v4BarToHeight(4),
                            new intakeCommand(),
                            new WaitCommand(200),
                            new v4BarToHeight(2),
                            new WaitCommand(300),
                            new v4BarToHeight(1),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
                            new followPath(toBackboardfromstack),
                            new WaitCommand(300),
                            new outtakeCommand(),
                            new WaitCommand(200),
                            new intakeCommand(),
                            new WaitCommand(400),
                            new outtakeCommand(),
                            new WaitCommand(300),
                            new stopIntake(),
                            new WaitCommand(300),
                            new pivotToTransferPosition(),
                            new pitchToTransferPosition(),
                            new WaitCommand(300),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new WaitCommand(200),
                            new WaitUntilCommand(extend),
                            new slideToRow(3),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new WaitUntilCommand(busy),
                            new WaitCommand(300),
                            new releaseLeftPixel(),
                            new releaseRightPixel(),
                            new WaitCommand(300),
                            new slideToRow(1),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),
                            new followPath(TostackSecondTime),
                            new flapDownStationary(),
                            new WaitUntilCommand(busy),
                            new intakeCommand(),
                            new followPath(TostrafeSecondTime),
                            new WaitCommand(200),
                            new v4BarToHeight(4),
                            new WaitCommand(200),
                            new v4BarToHeight(3),
                            new WaitCommand(100),
                            new v4BarToHeight(1),
                            new WaitCommand(200),
                            new WaitUntilCommand(busy),
                            new followPath(toBackboardSecondTime),
                            new outtakeCommand(),
                            new WaitCommand(200),
                            new intakeCommand(),
                            new WaitCommand(300),
                            new outtakeCommand(),
                            new WaitCommand(300),
                            new stopIntake(),
                            new WaitCommand(400),
                            new pivotToTransferPosition(),
                            new pitchToTransferPosition(),
                            new WaitCommand(400),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new WaitCommand(200),
                            new WaitUntilCommand(extend),
                            new slideToRow(3),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
                            new WaitCommand(100),
                            new releaseLeftPixel(),
                            new releaseRightPixel(),
                            new WaitCommand(200),
                            new slideToRow(1),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),
                            new WaitCommand(200),
                            new flapDownStationary()
                    )
            );

                            //above is new
//                            new followPath(toBackboard),
//                            new WaitUntilCommand(busy),
//                            new pitchToDropPosition(),
//                            new WaitCommand(250),
//                            new pivotToDropPosition(),
//                            new WaitCommand(1000),
//                            new releaseLeftPixel(),
//                            //below is new
//                            new releaseRightPixel()
                            //above is new


//                            new WaitUntilCommand(busy),
//                            new WaitCommand(500),
//                            new releaseLeftPixel(),
//                            new pivotToWaitPosition(),
//                            new pitchToWaitPosition(),
//
////                            new WaitCommand(100),
//
////                            new WaitCommand(100),
//                            new followPath(toBackboardMiddle),
//                            new WaitUntilCommand(busy),
//                            new followPath(toStackLeftFromBB),
//                            new WaitUntilCommand(busy)




            telemetry.addLine("Robot Initialized.");
            telemetry.addData("zone", zone);
            telemetry.update();
        }

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.visionPortal.stopStreaming();
        robot.read();
        robot.periodic();
        robot.write();
        telemetry.addData("is it busy", busy.getAsBoolean());
        telemetry.addData("zone", zone);
        telemetry.update();
    }
}