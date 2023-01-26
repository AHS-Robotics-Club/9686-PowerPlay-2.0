package org.firstinspires.ftc.teamcode.subsystems.teleop.SoMuchTesting.Slides;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.rr.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.rr.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.teleop.Intake.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.teleop.Intake.Lifts.LiftMotorGroup;
import org.firstinspires.ftc.teamcode.subsystems.teleop.Intake.Lifts.LiftSubsystem;

// NOTE: Face bot towards the plastic barrier, park bot between the tiles before the plastic barrier.
@Autonomous(name = "TestLifts")
public class TestLifts extends CommandOpMode {

    //motors
    private Motor leftLift, rightLLift ;
    private SimpleServo gripperServo;
    // Subsystems
    private MecanumDriveSubsystem mecanumDriveS;
    private LiftMotorGroup liftMotorGroup;
    private GripperSubsystem gripper;
    private LiftSubsystem lift;

    private final long LIFTUP= 2000;//2 seconds
    private final long LIFTDOWN  = 1000;
    // Extra Stuff
    private ElapsedTime time;

    // Pathing
    // Start Pose
    private Pose2d startPose = new Pose2d(0.0, -62.0, Math.toRadians(0.0));

    ;


    @Override
    public void initialize() {
        gripperServo = new SimpleServo(hardwareMap, "gripper", 0, 180);
        leftLift = new Motor(hardwareMap, "leftLift");
        rightLLift = new Motor(hardwareMap, "rightLift");
        //mecanumDriveS = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        liftMotorGroup = new LiftMotorGroup(leftLift,rightLLift);
        gripper =  new GripperSubsystem(gripperServo);
        lift = new LiftSubsystem(liftMotorGroup);
//        Trajectory traj0 = mecanumDriveS.trajectoryBuilder(startPose).
//                forward(46.0)
//                .build();

        if(isStopRequested()){
            return;


        }
        schedule(new WaitUntilCommand(this::isStarted).andThen(new SequentialCommandGroup(
                //new TrajectoryFollowerCommand(mecanumDriveS,traj0),
                new InstantCommand(() -> lift.runNormal()).raceWith(new WaitCommand(LIFTUP)),
               //; new InstantCommand(() -> gripper.release()),
                new InstantCommand(() -> lift.runNormalReverse()).raceWith(new WaitCommand(LIFTDOWN)),
                new InstantCommand(() -> gripper.lock())

        )));

    }
}