package org.firstinspires.ftc.teamcode.tests;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.MotorGroupTemp;

@Autonomous(name = "TestLift")
public class TestLift extends CommandOpMode {

    private Motor m1;
    private Motor m2;

    private MotorGroupTemp liftMotors;

    private  static double POWER = 0.5;

    @Override
    public void initialize() {
        m1 = new Motor(hardwareMap, "leftLift");
        m2 = new Motor(hardwareMap, "rightLift");

        liftMotors = new MotorGroupTemp(m1, m2);

        schedule(
                new WaitUntilCommand(this::isStarted)
                        .andThen(new InstantCommand(() -> liftMotors.set(POWER)))
                        .andThen(new WaitCommand(1000))
                        .andThen(new InstantCommand(() -> liftMotors.set(-POWER)))
                        .andThen(new WaitCommand(500))
                        .andThen(new InstantCommand(() -> liftMotors.set(0)))
                        .andThen(new RunCommand(() -> {
                            telemetry.addData("Motor Max RPM", liftMotors.getMaxRPM());
                            telemetry.addData("Motor Corrected Velocity", liftMotors.getCorrectedVelocity());
                            telemetry.update();
                        }
                        ))

        );



    }

}
