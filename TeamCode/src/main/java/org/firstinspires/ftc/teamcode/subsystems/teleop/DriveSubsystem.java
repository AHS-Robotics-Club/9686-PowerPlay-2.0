package org.firstinspires.ftc.teamcode.subsystems.teleop;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveSubsystem extends SubsystemBase {
    private Motor frontLeft, frontRight, backLeft, backRight;
    private RevIMU imu;
    private MecanumDrive mecanumDrive;

    public DriveSubsystem(Motor fL, Motor fR, Motor bL, Motor bR, RevIMU imu){
        frontLeft = fL;
        frontRight = fR;
        backLeft = bL;
        backRight = bR;

        this.imu = imu;

        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    // Method which runs the subsystem
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        if(frontLeft != null && frontRight != null && backLeft != null && backRight != null)
            // mecanumDrive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, imu.getRotation2d().getDegrees());
            mecanumDrive.driveRobotCentric(-strafeSpeed, -forwardSpeed, -turnSpeed, true);
    }
}
