package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.hangCommand;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToDropPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToRearrangePosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToWaitPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToDropPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToRearrangePosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToWaitPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.setRollAngle;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.slideToRow;
import org.firstinspires.ftc.teamcode.common.commands.droneLaunch;
import org.firstinspires.ftc.teamcode.common.commands.droneToHeightUp;
import org.firstinspires.ftc.teamcode.common.commands.droneToWait;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.flapDownStationary;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.outtakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.stopIntake;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.v4BarToHeight;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.commands.hangReleaseCommand;


@TeleOp
public class teleopRED extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private GamepadEx gamepadDrivetrain;
    private GamepadEx gamepadMechanism;
    private Vector driveVector;
    private Vector headingVector;

    private double[] rollAngles = {0, Math.toRadians(60), Math.toRadians(90), Math.toRadians(120), Math.toRadians(180), Math.toRadians(210),Math.toRadians(270) ,Math.toRadians(300)};
    private int rollIndex = 0;
    private int targetRow = 1;
    private int fourBarHeight=1;
    private boolean isLeftDropped = false;
    private boolean isRightDropped = false;
    private boolean transferred = false;
    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = false;
        gamepadDrivetrain = new GamepadEx(gamepad1);
        gamepadMechanism = new GamepadEx(gamepad2);
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);

        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(new Runnable() {
            @Override
            public void run() {
                targetRow+=1;
            }
        }));
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(new Runnable() {
            @Override
            public void run() {
                if(targetRow!=0)targetRow-=1;
            }
        }));
//        gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(new Runnable() {
//            @Override
//            public void run() {
//                targetRow=8;
//            }
//        }));
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(new Runnable() {
            @Override
            public void run() {
                if(rollIndex != 0) rollIndex-=1;
                else rollIndex = 7;
            }
        }));
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(new Runnable() {
            @Override
            public void run() {
                if(rollIndex != 7) rollIndex+=1;
                else rollIndex = 0;
            }
        }));

        gamepadMechanism.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SequentialCommandGroup(new outtakeCommand(),new v4BarToHeight(5)));
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SequentialCommandGroup(new intakeCommand(),new v4BarToHeight(1)));

        gamepadMechanism.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        new stopIntake(),
                        new WaitCommand(200),
                        new pivotToTransferPosition(),
                        new WaitCommand(250),
                        new pitchToTransferPosition(),
                        new WaitCommand(250),
                        new grabLeftPixel(),
                        new grabRightPixel()
                )
        );
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new SequentialCommandGroup(
                new pitchToDropPosition(),
                new WaitCommand(80),
                new pivotToDropPosition()
        ));
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new SequentialCommandGroup(
                new pivotToRearrangePosition(), new pitchToRearrangePosition()
        ));
        gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SequentialCommandGroup(
                new InstantCommand(new Runnable() {
                    @Override
                    public void run() {
                        targetRow=8;
                    }
                }), new flapDownStationary(), new pitchToDropPosition(), new pivotToDropPosition()));
//        gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.B).whenPressed(new hangCommand());
        gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.B).whenPressed(new SequentialCommandGroup(new hangCommand(),
                new InstantCommand(new Runnable() {
                    @Override
                    public void run() {
                        targetRow=2;
                    }
                }),new WaitCommand(4500),new hangReleaseCommand()));

        gamepadMechanism.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(new Runnable() {
            @Override
            public void run() {
                fourBarHeight-=1;
                fourBarHeight= (int)MathUtils.clamp(fourBarHeight,1,5);
            }
        }));
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new InstantCommand(new Runnable() {
            @Override
            public void run() {
                fourBarHeight+=1;
                fourBarHeight=(int) MathUtils.clamp(fourBarHeight,1,5);
            }
        }));



        robot.read();
        robot.periodic();
        robot.write();
        driveVector = new Vector();
        headingVector = new Vector();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }

    }
    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();
        if(robot.deposit.getPitchState() == depositSubsystem.armState.drop && robot.intake.getState() != intakeSubsystem.intakeState.hang){
            driveVector.setOrthogonalComponents(-gamepadDrivetrain.getLeftY()*0.4, -gamepadDrivetrain.getRightY()*0.75);
        }
        else{driveVector.setOrthogonalComponents(-gamepadDrivetrain.getLeftY(), -gamepadDrivetrain.getRightY());}
        driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
        driveVector.rotateVector(robot.follower.getPose().getHeading());
        headingVector.setComponents((gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)-gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))*0.5, robot.follower.getPose().getHeading());
        robot.follower.setMovementVectors(robot.follower.getCentripetalForceCorrection(), headingVector, driveVector);

        if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {CommandScheduler.getInstance().schedule(new releaseLeftPixel()); isLeftDropped = true;}
        if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {CommandScheduler.getInstance().schedule(new releaseRightPixel()); isRightDropped = true;}
        if(isLeftDropped && isRightDropped && (robot.deposit.getPivotState() != depositSubsystem.armState.wait) && (robot.deposit.getPitchState() != depositSubsystem.armState.wait)){
            isLeftDropped = false;
            isRightDropped = false;
            transferred = false; 
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new WaitCommand(400),new pitchToWaitPosition(), new pivotToWaitPosition(), new WaitCommand(400), new setRollAngle(0),new InstantCommand(new Runnable() {
                @Override
                public void run() {
                    rollIndex = 0;
                }
            })
            ));

        }
        if(robot.intake.getLeftPixel() && robot.intake.getRightPixel() && !transferred){
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new stopIntake()
//                            new WaitCommand(200),
//                            new pivotToTransferPosition(),
//                            new WaitCommand(250),
//                            new pitchToTransferPosition(),
//                            new WaitCommand(250),
//                            new grabLeftPixel(),
//                            new grabRightPixel()
                    )
            );
            transferred = true;
        }

        if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get()) {
            isLeftDropped = false;
            isRightDropped = false;
        }


        if (((robot.deposit.getPivotState() == depositSubsystem.armState.rearrange) || (robot.deposit.getPivotState() == depositSubsystem.armState.drop))) {
            if (robot.deposit.getSlideTargetRow() != targetRow) {
                CommandScheduler.getInstance().schedule(new slideToRow(targetRow));
            }
        }
        else {
            if (robot.deposit.getSlideTargetRow() != 0) {
                CommandScheduler.getInstance().schedule(new slideToRow(0));
            }
        }

        if ((robot.deposit.getPitchState() == depositSubsystem.armState.drop) && (robot.deposit.getPivotState() == depositSubsystem.armState.drop) && rollAngles[rollIndex]!=robot.deposit.getRollAngle()){
            CommandScheduler.getInstance().schedule(new setRollAngle(rollAngles[rollIndex]));
        }

        if (((robot.intake.getState() == intakeSubsystem.intakeState.outtake) || (robot.intake.getState() == intakeSubsystem.intakeState.intake)) && robot.intake.targetStackHeight!=fourBarHeight){
            CommandScheduler.getInstance().schedule(new v4BarToHeight(fourBarHeight));
        }

        if (gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get() && gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.X).get())
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new droneToHeightUp(),
                            new WaitCommand(1000),
                            new droneLaunch(),
                            new WaitCommand(200),
                            new droneToHeightUp(),
                            new WaitCommand(100),
                            new droneLaunch(),
                            new WaitCommand(100),
                            new droneToWait()
                    )
            );
//        if (robot.intake.getLeftPixel() && robot.intake.getState() == intakeSubsystem.intakeState.intake){
//            robot.leftRed.setState(false);
//            robot.leftFrontRed.setState(false);
//            robot.leftGreen.setState(true);
//            robot.leftFrontgreen.setState(true);
//        }
//        else{
//            robot.leftRed.setState(true);
//            robot.leftFrontRed.setState(true);
//            robot.leftGreen.setState(false);
//            robot.leftFrontgreen.setState(false);
//        }
//        if (robot.intake.getRightPixel()){
//            robot.rightRed.setState(false);
//            robot.rightFrontRed.setState(false);
//            robot.rightGreen.setState(true);
//            robot.rightFrontgreen.setState(true);
//        }
//        else{
//            robot.rightRed.setState(true);
//            robot.rightFrontRed.setState(true);
//            robot.rightGreen.setState(false);
//            robot.rightFrontgreen.setState(false);
//        }

    }
}