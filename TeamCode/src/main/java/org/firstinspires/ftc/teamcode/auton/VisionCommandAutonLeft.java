package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.commands.rr.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.rr.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.teleop.Intake.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.teleop.Intake.Lifts.LiftMotorGroup;
import org.firstinspires.ftc.teamcode.subsystems.teleop.Intake.Lifts.LiftSubsystem;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Left Park Auton")
public class VisionCommandAutonLeft extends CommandOpMode {
    private OpenCvCamera monkeyCam;
    private AprilTagPipeline detector;

    // Drivetrain
    private SampleMecanumDrive lilSailaja;
    private MecanumDriveSubsystem sailaja;

    private Motor leftLift, rightLift;
    private LiftMotorGroup lifts;
    private SimpleServo gripper;

    private LiftSubsystem liftSubsystem;
    private GripperSubsystem gripperSubsystem;

    private Pose2d startPose = new Pose2d(0, 0, 0);

    // CAMERA CALIBS
    double fx = 1430;
    double fy = 1430;
    double cx = 430;
    double cy = 620;

    // CONSTANTS (IN METERS)
    final double tagSize = 0.166;

    // Tag ID's
    final int LEFT = 1;
    final int MIDDLE = 2;
    final int RIGHT = 3;

    final double travelDist = 2;
    final int shortSleep = 5000;
    final int longSleep = 10000;
    final double upPower = 1.0;
    final double downPower = 0.8;

    AprilTagDetection tag = null;

    @Override
    public void initialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        monkeyCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "udaya"), cameraMonitorViewId);
        detector = new AprilTagPipeline(tagSize, fx, fy, cx, cy);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lilSailaja = new SampleMecanumDrive(hardwareMap);
        sailaja = new MecanumDriveSubsystem(lilSailaja, true);

        leftLift = new Motor(hardwareMap, "leftLift");
        rightLift = new Motor(hardwareMap, "rightLift");
        lifts = new LiftMotorGroup(leftLift, rightLift);
        gripper = new SimpleServo(hardwareMap, "gripper", 0, 180);

        liftSubsystem = new LiftSubsystem(lifts);
        gripperSubsystem = new GripperSubsystem(gripper);

        //region trajs
        Trajectory traj0 = sailaja.trajectoryBuilder(startPose)
                .strafeRight(travelDist * 2)
                .build();
        Trajectory traj1 = sailaja.trajectoryBuilder(traj0.end())
                .forward(travelDist / 5)
                .build();

        // APRIL TAG TRAJS
        Trajectory resetForPark = sailaja.trajectoryBuilder(traj1.end())
                .back(travelDist / 5)
                .build();
        Trajectory resetToCone = sailaja.trajectoryBuilder(resetForPark.end())
                .strafeLeft(travelDist)
                .build();
        // GO LEFT
        Trajectory leftPark = sailaja.trajectoryBuilder(resetToCone.end())
                .forward(travelDist)
                .build();
        // GO RIGHT
        Trajectory rightPark = sailaja.trajectoryBuilder(resetToCone.end())
                .back(travelDist)
                .build();
        //endregion

        monkeyCam.setPipeline(detector);
        monkeyCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened()
            {
                monkeyCam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        /*
        Command based specific, need to figure this out 😭☠️
         */
        while(!isStarted() && !isStopRequested()){
            ArrayList<AprilTagDetection> currentDetections = detector.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        this.tag = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    telemetry.addLine(String.format("\nDetected tag ID=%d", this.tag.id));
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(this.tag == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        telemetry.addLine(String.format("\nDetected tag ID=%d",this.tag.id));
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(this.tag == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    telemetry.addLine(String.format("\nDetected tag ID=%d",this.tag.id));
                }

            }

            telemetry.update();
        }

        schedule(new RunCommand(() -> {
            if (this.tag == null) {
                new InstantCommand(() -> gripperSubsystem.lock())
                        .andThen(new TrajectoryFollowerCommand(sailaja, traj0))
                        .andThen(new TrajectoryFollowerCommand(sailaja, traj1))
                        .andThen(new InstantCommand(() -> lifts.set(upPower)))
                        .andThen(new WaitCommand(shortSleep))
                        .andThen(new InstantCommand(() -> lifts.stopMotor()))
                        .andThen(new InstantCommand(() -> gripperSubsystem.release()))
                        .andThen(new TrajectoryFollowerCommand(sailaja, resetForPark))
                        .andThen(new InstantCommand(() -> lifts.set(-downPower)))
                        .andThen(new WaitCommand(shortSleep))
                        .andThen(new TrajectoryFollowerCommand(sailaja, resetToCone));
                //default trajectory here if preferred
            } else if (this.tag.id == LEFT) {
                //left trajectory
                new InstantCommand(() -> gripperSubsystem.lock())
                        .andThen(new TrajectoryFollowerCommand(sailaja, traj0))
                        .andThen(new TrajectoryFollowerCommand(sailaja, traj1))
                        .andThen(new InstantCommand(() -> lifts.set(upPower)))
                        .andThen(new WaitCommand(shortSleep))
                        .andThen(new InstantCommand(() -> lifts.stopMotor()))
                        .andThen(new InstantCommand(() -> gripperSubsystem.release()))
                        .andThen(new TrajectoryFollowerCommand(sailaja, resetForPark))
                        .andThen(new InstantCommand(() -> lifts.set(-downPower)))
                        .andThen(new WaitCommand(shortSleep))
                        .andThen(new TrajectoryFollowerCommand(sailaja, resetToCone))
                        .andThen(new TrajectoryFollowerCommand(sailaja, leftPark));

            } else if (this.tag.id == MIDDLE) {
                //Middle Trajectory
                new InstantCommand(() -> gripperSubsystem.lock())
                        .andThen(new TrajectoryFollowerCommand(sailaja, traj0))
                        .andThen(new TrajectoryFollowerCommand(sailaja, traj1))
                        .andThen(new InstantCommand(() -> lifts.set(upPower)))
                        .andThen(new WaitCommand(shortSleep))
                        .andThen(new InstantCommand(() -> lifts.stopMotor()))
                        .andThen(new InstantCommand(() -> gripperSubsystem.release()))
                        .andThen(new TrajectoryFollowerCommand(sailaja, resetForPark))
                        .andThen(new InstantCommand(() -> lifts.set(-downPower)))
                        .andThen(new WaitCommand(shortSleep))
                        .andThen(new TrajectoryFollowerCommand(sailaja, resetToCone));
            } else {
                //right trajectory
                new InstantCommand(() -> gripperSubsystem.lock())
                        .andThen(new TrajectoryFollowerCommand(sailaja, traj0))
                        .andThen(new TrajectoryFollowerCommand(sailaja, traj1))
                        .andThen(new InstantCommand(() -> lifts.set(upPower)))
                        .andThen(new WaitCommand(shortSleep))
                        .andThen(new InstantCommand(() -> lifts.stopMotor()))
                        .andThen(new InstantCommand(() -> gripperSubsystem.release()))
                        .andThen(new TrajectoryFollowerCommand(sailaja, resetForPark))
                        .andThen(new InstantCommand(() -> lifts.set(-downPower)))
                        .andThen(new WaitCommand(shortSleep))
                        .andThen(new TrajectoryFollowerCommand(sailaja, resetToCone))
                        .andThen(new TrajectoryFollowerCommand(sailaja, rightPark));
            }
        }));


    }
}
