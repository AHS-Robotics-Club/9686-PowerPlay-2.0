package org.firstinspires.ftc.teamcode.subsystems.teleop.SoMuchTesting.Slides;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.subsystems.teleop.Intake.Lifts.LiftMotorGroup;

public class TestPLift extends SubsystemBase {
    private LiftMotorGroup lifts;

    private double kP = 1.0;

    /**
     * Constructs the lift subsystem using a lift motor group + a kP value
     * @param liftMotorGroup    Lift Motors; should be in a motor group. Declare lift motors as
     *                          LiftMotorGroup lifts = new LiftMotorGroup(leftLift, rightLift).
     * -----------------------------------------------------------------------------------------
     * @param kP    Proportional gain; use a default value of 1.0, but tune to get exact
     *              value. Aim to get 3-4 decimal places of precision.
     **/
    public TestPLift(LiftMotorGroup liftMotorGroup, double kP){
        lifts = liftMotorGroup;
        this.kP = kP;
    }

    /**
     * Runs the lift subsystem to a position
     *
     * @param pos    I have no clue how this stuff works j experiment with different values until it works ig
     * @param tolerance Keep default as 1, you only need to change it if the subsystem like dies
     */
    public void runToPos(int pos, double tolerance){
        double output;

        com.arcrobotics.ftclib.controller.PController posCtrl = new com.arcrobotics.ftclib.controller.PController(kP);

        posCtrl.setSetPoint(pos);
        posCtrl.setTolerance(tolerance);

        while(!posCtrl.atSetPoint()){
            output = posCtrl.calculate(lifts.getCurrentPosition());
            lifts.set(output);
        }
        lifts.stopMotor();
    }

}
