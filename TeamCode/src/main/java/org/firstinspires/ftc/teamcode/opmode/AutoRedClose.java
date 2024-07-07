package org.firstinspires.ftc.teamcode.opmode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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
import org.firstinspires.ftc.teamcode.common.commands.armCommands.setRollAngle;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.slideToRow;
import org.firstinspires.ftc.teamcode.common.commands.droneToWait;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.followPath;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.flapDownStationary;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeAutoState;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.outtakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.stopIntake;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.v4BarToHeight;

import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;

import java.util.function.BooleanSupplier;


@Autonomous
public class AutoRedClose extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private Path toSpikeLeft;
    private Path toSpikeMiddle;
    private Path toSpikeRight;
    private Path toStackRight;
    private Path toStackMiddle;
    private Path toStrafeAtStackRight;
    private Path toStrafeAtStackMiddle;
    private Path toBackboardRight;
    private Path toBackboardMiddle;
    private Path toBackboardLeft;
    private Path toTruss;
    private Path toTrussNearStack;
    private Path toStackLeft;
    private Path toStrafeAtStackLeft;
    private Path toTrussNearStackback;
    private Path totrussback;
    private Path toStackLeftFromBB;
    private Path toBackboardFromStack;
    private Path fromBBtoStackfullcurve;
    private Path tobackboardFromStack;
    private Path tocurvetostackBack;
    private Path tocurvetostack;
    public boolean closed = false;
    private Path toPark;
    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() >29.3;
    private int zone;
    private Location randomization;

    @Override
    public void initialize() {


        telemetry.setMsTransmissionInterval(50);

        fromBBtoStackfullcurve = new Path(new BezierCurve(new Point(23,-37,Point.CARTESIAN),(new Point(1,-7,Point.CARTESIAN)),(new Point(1,15,Point.CARTESIAN)),new Point(1,21,Point.CARTESIAN),(new Point(5  ,44,Point.CARTESIAN)),(new Point(10,47 ,Point.CARTESIAN)),(new Point(15  ,55 ,Point.CARTESIAN)),(new Point(26, 66.5,Point.CARTESIAN))));
        fromBBtoStackfullcurve.setConstantHeadingInterpolation(Math.toRadians(-95));

        tobackboardFromStack = new Path(new BezierLine((new Point(3,0,Point.CARTESIAN)),new Point(23,-35.5,Point.CARTESIAN)));
        tobackboardFromStack.setConstantHeadingInterpolation(Math.toRadians(-95));

        toPark = new Path(new BezierLine((new Point(18.5,-37,Point.CARTESIAN)),new Point(0,-37,Point.CARTESIAN)));
        toPark.setConstantHeadingInterpolation(Math.toRadians(-90));

        toSpikeMiddle = new Path(new BezierLine(new Point(4, 0,Point.CARTESIAN),new Point(33, -17,Point.CARTESIAN)));
        toSpikeMiddle.setConstantHeadingInterpolation(Math.toRadians(90));

        toSpikeLeft = new Path(new BezierCurve(new Point(4, 0,Point.CARTESIAN), new Point(13,-13,Point.CARTESIAN), new Point(22.5, -2.7,Point.CARTESIAN)));
        toSpikeLeft.setConstantHeadingInterpolation(Math.toRadians(90));

        toSpikeRight = new Path(new BezierLine(new Point(4, 0,Point.CARTESIAN), new Point(23, -22,Point.CARTESIAN)));
        toSpikeRight.setConstantHeadingInterpolation(Math.toRadians(90));

        toBackboardMiddle = new Path(new BezierLine(new Point(21, 13, Point.CARTESIAN),new Point(23, -37 ,Point.CARTESIAN))); //new Point(21, 0, Point.CARTESIAN),
        toBackboardMiddle.setConstantHeadingInterpolation(Math.toRadians(-90));

        toBackboardLeft = new Path(new BezierLine((new Point(21.8, -2.7, Point.CARTESIAN)),(new Point(29, -37,Point.CARTESIAN))));
        toBackboardLeft.setConstantHeadingInterpolation(Math.toRadians(-90));

        toBackboardRight = new Path(new BezierLine(new Point(23,-24,Point.CARTESIAN),new Point(18.5,-36.5,Point.CARTESIAN)));
        toBackboardRight.setConstantHeadingInterpolation(Math.toRadians(-90));

        toTruss = new Path(new BezierLine(new Point(23,-37,Point.CARTESIAN),new Point(3,-8,Point.CARTESIAN)));
        toTruss.setConstantHeadingInterpolation(Math.toRadians(-90));

        toTrussNearStack = new Path(new BezierLine(new Point(3,0,Point.CARTESIAN),new Point(3,45,Point.CARTESIAN)));
        toTrussNearStack.setConstantHeadingInterpolation(Math.toRadians(-90));

        toStackLeft = new Path(new BezierLine(new Point(3,45,Point.CARTESIAN),new Point(22,67,Point.CARTESIAN)));
        toStackLeft.setConstantHeadingInterpolation(Math.toRadians(-135));
        toStrafeAtStackLeft = new Path(new BezierLine(new Point(26,67,Point.CARTESIAN),new Point(32,67,Point.CARTESIAN)));
        toStrafeAtStackLeft.setConstantHeadingInterpolation(Math.toRadians(-93));

        toStackLeftFromBB = new Path(new BezierCurve((new Point(43,-35,Point.CARTESIAN)),(new Point(30,2,Point.CARTESIAN)),(new Point(-27,4,Point.CARTESIAN)),(new Point(-57, -19,Point.CARTESIAN))));
        toStackLeftFromBB.setConstantHeadingInterpolation(0);

        tocurvetostack = new Path(new BezierCurve((new Point(3,0,Point.CARTESIAN)),(new Point(5,59,Point.CARTESIAN)),(new Point(22, 66.5,Point.CARTESIAN))));
        tocurvetostack.setConstantHeadingInterpolation(Math.toRadians(-93));


        tocurvetostackBack = new Path(new BezierCurve((new Point(22,67,Point.CARTESIAN)),(new Point(5,60,Point.CARTESIAN)),(new Point(3, 0,Point.CARTESIAN))));
        tocurvetostackBack.setConstantHeadingInterpolation(Math.toRadians(-93));

        toTrussNearStackback = new Path(new BezierLine(new Point(22,67,Point.CARTESIAN),new Point(3,45,Point.CARTESIAN)));
        toTrussNearStackback.setConstantHeadingInterpolation(Math.toRadians(-94));

        totrussback = new Path(new BezierLine(new Point(3,45,Point.CARTESIAN),new Point(3,0,Point.CARTESIAN)));
        toTrussNearStackback.setConstantHeadingInterpolation(Math.toRadians(-94));

//        toStackRight = new Path(new BezierCurve(new Point(23, -36.25, Point.CARTESIAN), new Point(56.6, -33,Point.CARTESIAN),new Point(65, 0,Point.CARTESIAN),(new Point(48.17, 68.7, Point.CARTESIAN))));
//        toStackRight.setConstantHeadingInterpolation(Math.toRadians(-85));
//        toStackRight.setReversed(true);
//        toStrafeAtStackRight = new Path(new BezierLine((new Point(48.17,68.2,Point.CARTESIAN)),(new Point(45.6,68.5,Point.CARTESIAN))));
//        toStrafeAtStackRight.setConstantHeadingInterpolation(Math.toRadians(-85));
//
//        toStackMiddle = new Path(new BezierCurve(new Point(23, -36, Point.CARTESIAN), new Point(56.6, -33,Point.CARTESIAN),new Point(65, 0,Point.CARTESIAN),(new Point(34, 68, Point.CARTESIAN))));
//        toStackMiddle.setConstantHeadingInterpolation(Math.toRadians(-85));
//        toStackMiddle.setReversed(true);
//        toStrafeAtStackMiddle = new Path(new BezierLine((new Point(34,67.5,Point.CARTESIAN)),(new Point(30 ,67.5,Point.CARTESIAN))));
//        toStrafeAtStackMiddle.setConstantHeadingInterpolation(Math.toRadians(-85));
//
//        toBackboardFromStack = new Path(new BezierCurve((new Point(39.3, 68.65, Point.CARTESIAN)),(new Point(65, 0,Point.CARTESIAN)),(new Point(56.6, -33,Point.CARTESIAN)),(new Point(23, -37.5,Point.CARTESIAN))));
//        toBackboardFromStack.setConstantHeadingInterpolation(Math.toRadians(-90));
        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        CenterstageConstants.ALLIANCE = Location.RED;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);


//        robot.follower.setStartingPose(new Pose(-60,34,Math.toRadians(90)));
        while (opModeInInit()) {

            zone= robot.propDetectionPipeline.detectZone();
            CommandScheduler.getInstance().reset();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new followPath(CenterstageConstants.getPath(zone,toSpikeLeft,toSpikeMiddle,toSpikeRight)),
                                    new SequentialCommandGroup(
                                            new pitchToTransferPosition(),
                                            new pivotToTransferPosition(),
                                            new WaitCommand(200),
                                            new grabRightPixel(),
                                            new grabLeftPixel(),
                                            new WaitCommand(700),
                                            new pitchToSpikeDrop(),
                                            new WaitCommand(100),
                                            new PivotToSpikeDrop(),
                                            new WaitCommand(300)
                                    ),
                                    new WaitUntilCommand(busy)
                            ),
                            new releaseRightPixel(),
                            new WaitCommand(100),
                            new stopIntake(),
                            new ParallelCommandGroup(
                                    new followPath(CenterstageConstants.getPath(zone,toBackboardLeft,toBackboardMiddle,toBackboardRight)),
                                    new SequentialCommandGroup(
                                            new pitchToDropPosition(),
                                            new pivotToDropPosition()
                                    ),
                                    new WaitUntilCommand(busy)),
                            new WaitCommand(500),
                            new releaseLeftPixel(),
                            new pitchToWaitPosition(),
                            new WaitCommand(100),
                            new pivotToWaitPosition(),
                            new flapDownStationary(),
                            new followPath(fromBBtoStackfullcurve),
                            new WaitUntilCommand(busy),
                            new intakeCommand(),
                            new followPath(toStrafeAtStackLeft),
                            new v4BarToHeight(5),
                            new WaitCommand(200),
                            new v4BarToHeight(4),
                            new WaitCommand(400),
                            new v4BarToHeight(3),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
                            new followPath(tocurvetostackBack),
                            new outtakeCommand(),
                            new WaitCommand(150),
                            new intakeCommand(),
                            new WaitCommand(500),
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
                            new WaitUntilCommand(busy),
                            new followPath(tobackboardFromStack),
                            new WaitCommand(300),
                            new slideToRow(3),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
                            new releaseLeftPixel(),
                            new releaseRightPixel(),
                            new WaitCommand(200),
                            new slideToRow(1),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),
                            new WaitCommand(200),
                            new followPath(fromBBtoStackfullcurve),
                            new flapDownStationary(),
                            new WaitUntilCommand(busy),
                            new intakeCommand(),
                            new followPath(toStrafeAtStackLeft),
                            new v4BarToHeight(3),
                            new WaitCommand(300),
                            new v4BarToHeight(2),
                            new WaitCommand(300),
                            new v4BarToHeight(1),
                            new WaitCommand(200),
                            new WaitUntilCommand(busy),
                            new followPath(tocurvetostackBack),
                            new outtakeCommand(),
                            new WaitCommand(200),
                            new intakeCommand(),
                            new WaitCommand(300),
                            new outtakeCommand(),
                            new WaitCommand(150),
                            new stopIntake(),
                            new WaitCommand(400),
                            new pivotToTransferPosition(),
                            new pitchToTransferPosition(),
                            new WaitCommand(400),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new WaitCommand(200),
                            new WaitUntilCommand(busy),
                            new followPath(tobackboardFromStack),
                            new WaitCommand(300),
                            new slideToRow(3),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
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
        telemetry.addData("joemamaobama",robot.getCameraState());

    }
}
////                            new pivotToWaitPosition(),
////                            new pitchToWaitPosition(),
//
//                            new followPath(toStackRight),
////                            new v4BarToHeight(5),
////                            new outtakeCommand(),
//                            new WaitUntilCommand(busy),
//                            new WaitCommand(350),
//                            new followPath(toStrafeAtStackRight),
////                            new v4BarToHeight(4),
//                            new WaitCommand(150),
////                            new v4BarToHeight(3),
//                            new WaitCommand(150),
//
////                            new v4BarToHeight(5),
//                            new WaitCommand(150),
////                            new v4BarToHeight(4),
//                            new WaitCommand(150),
////                            new v4BarToHeight(3),
//                            new WaitCommand(150),
//
//                            new WaitUntilCommand(busy),
//                            new WaitCommand(150),
//                            new v4BarToHeight(4),
//                            new ParallelRaceGroup(
//                                    new WaitUntilCommand(pixels),
//                                    new WaitCommand(1500)
//                            ),
//
//                            new stopIntake(),
//                            //   new WaitUntilCommand(pixels),
//                            new intakeCommand(),
//                            new WaitCommand(200),
//                            new stopIntake(),
//                            new WaitCommand(400),
//                            new pivotToTransferPosition(),
//                            new WaitCommand(200),
//                            new pitchToTransferPosition(),
//
//                            new followPath(toBackboardFromStack),
//                            new intakeCommand(),
//                            new WaitCommand(500),
//                            new stopIntake(),
//                            new grabLeftPixel(),
//                            new grabRightPixel(),
//                            new WaitUntilCommand(busy),
//                            new pitchToDropPosition(),
//                            new WaitCommand(80),
//                            new pivotToDropPosition(),
//                            new slideToRow(3),
//                            new WaitCommand(600),
//                            new releaseRightPixel(),
//                            new releaseLeftPixel(),
//
//                            new slideToRow(1),
//                            new pivotToWaitPosition(),
//                            new pitchToWaitPosition(),
//
//                            new followPath(toStackMiddle),
//                            new v4BarToHeight(5),
//                            new outtakeCommand(),
//                            new WaitUntilCommand(busy),
//                            new WaitCommand(750),
//                            new followPath(toStrafeAtStackMiddle),
//                            new v4BarToHeight(4),
//                            new WaitCommand(50),
//                            new v4BarToHeight(3),
//                            new WaitCommand(50),
//
//                            new v4BarToHeight(5),
//                            new WaitCommand(50),
//                            new v4BarToHeight(4),
//                            new WaitCommand(50),
//                            new v4BarToHeight(3),
//                            new WaitCommand(50),
//
//                            new WaitUntilCommand(busy),
//                            new WaitCommand(50),
//                            new v4BarToHeight(4),
//                            new ParallelRaceGroup(
//                                    new WaitUntilCommand(pixels),
//                                    new WaitCommand(1500)
//                            ),
//
//                            new intakeCommand(),
//                            new WaitCommand(200),
//                            new stopIntake(),
//                            new WaitCommand(400),
//                            new pivotToTransferPosition(),
//                            new WaitCommand(200),
//                            new pitchToTransferPosition(),
//
//                            new followPath(toBackboardFromStack),
//                            new WaitCommand(500),
//                            new grabLeftPixel(),
//                            new grabRightPixel(),
//                            new WaitUntilCommand(busy),
//                            new pitchToDropPosition(),
//                            new WaitCommand(80),
//                            new pivotToDropPosition(),
//                            new slideToRow(3),
//                            new WaitCommand(600),
//                            new releaseRightPixel(),
//                            new releaseLeftPixel()
