package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "park beta")
public class VisionParkingOnlyAuton extends LinearOpMode {
    private OpenCvCamera monkeyCam;
    private AprilTagPipeline detector;

    // Drivetrain
//    private Motor fL, fR, bL, bR;
//    private RevIMU imu;
//    private DriveSubsystem drive;
    private SampleMecanumDrive endMySuffering;
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

    AprilTagDetection tag = null;


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        monkeyCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "udaya"), cameraMonitorViewId);
        detector = new AprilTagPipeline(tagSize, fx, fy, cx, cy);

//        fL = new Motor(hardwareMap, "frontLeft");
//        fR = new Motor(hardwareMap, "frontRight");
//        bL = new Motor(hardwareMap, "backLeft");
//        bR = new Motor(hardwareMap, "backRight");
//
//        imu = new RevIMU(hardwareMap);
//
//        drive = new DriveSubsystem(fL, fR, bL, bR, imu);

        endMySuffering = new SampleMecanumDrive(hardwareMap);
        endMySuffering.setPoseEstimate(startPose);

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
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
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

        /* Actually do something useful */
        if(this.tag == null){
            //default trajectory here if preferred
            telemetry.addData("Hello World", -1);
            telemetry.update();
            endMySuffering.followTrajectorySequence(
                    endMySuffering.trajectorySequenceBuilder(startPose)
                            .forward(1.8)
                            .build()
            );

        }else if(this.tag.id == LEFT){
            telemetry.addData("Hello World", 0);
            telemetry.update();
            endMySuffering.followTrajectorySequence(
                    endMySuffering.trajectorySequenceBuilder(startPose)
                            .strafeLeft(3.7)
                            .forward(1.8)
                            .build()
            );
            //left trajectory
        }else if(this.tag.id == MIDDLE){
            //middle trajectory
            telemetry.addData("Hello World", 1);
            telemetry.update();
            endMySuffering.followTrajectorySequence(
                    endMySuffering.trajectorySequenceBuilder(startPose)
                            .forward(1.8)
                            .build()
            );
        }else{
            //right trajectory
            telemetry.addData("Hello World", 2);
            telemetry.update();
            endMySuffering.followTrajectorySequence(
                    endMySuffering.trajectorySequenceBuilder(startPose)
                            .strafeRight(3.7)
                            .forward(1.8)
                            .build()
            );
        }


    }
}