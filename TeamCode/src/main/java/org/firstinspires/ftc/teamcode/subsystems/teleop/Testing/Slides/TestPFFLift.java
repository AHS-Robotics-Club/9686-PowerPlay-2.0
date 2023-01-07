package org.firstinspires.ftc.teamcode.subsystems.teleop.Testing.Slides;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;

import org.firstinspires.ftc.teamcode.subsystems.teleop.Intake.Lifts.LiftMotorGroup;

public class TestPFFLift extends SubsystemBase {
    private LiftMotorGroup lifts;

    private ElevatorFeedforward liftFFControl;
    private PController liftPControl;

    /**
     * Constructs the lift subsystem using a lift motor group + a kP value
     * @param liftMotorGroup    Lift Motors; should be in a motor group. Declare lift motors as
     *                          LiftMotorGroup lifts = new LiftMotorGroup(leftLift, rightLift).
     * -----------------------------------------------------------------------------------------
     * @param kP                Proportional Gain; Get this value with tuning
     * -----------------------------------------------------------------------------------------
     * @param kS                Static Friction; impossible to tune so just find a value that doesn't screw up the entire subsystem
     * @param kG                Gravity; -10 or -9.8 should work (if it doesn't, use positive versions)
     * @param kV                Max desirable velocity, 0.8-0.9 is the sweet spot
     * @param kA                Max desirable acceleration; any value works, just make sure this doesn't break the slides
     */
    public TestPFFLift(LiftMotorGroup liftMotorGroup, double kP, double kS, double kG, double kV, double kA){
        lifts = liftMotorGroup;

        liftFFControl = new ElevatorFeedforward(kS, kG, kV, kA);
        liftPControl = new PController(kP);
    }

    /**
     * Runs the lift subsystem to a position
     * 
     * @param runPos    I have no clue how this stuff works j experiment with different values until it works ig
     * @param tolerance Keep default as 1, you only need to change it if the subsystem like dies
     */
    public void runToPos(int runPos, double tolerance){
        double output;
        
        liftPControl.setSetPoint(runPos);
        liftPControl.setTolerance(tolerance);

        while(!liftPControl.atSetPoint()){
            output = liftPControl.calculate(lifts.getCurrentPosition());
            lifts.set(liftFFControl.calculate(output));
        }
        lifts.stopMotor();
    }


}
