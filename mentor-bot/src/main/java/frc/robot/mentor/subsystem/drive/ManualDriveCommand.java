package frc.robot.mentor.subsystem.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleSupplier;

public class ManualDriveCommand extends CommandBase {

    private final DifferentialDrive differentialDrive;
    private final DoubleSupplier speed;
    private final DoubleSupplier turn;

    public ManualDriveCommand(Subsystem subsystem, MotorController leftMotor, MotorController rightMotor, DoubleSupplier speed, DoubleSupplier turn) {
        addRequirements(subsystem);
        // we invert our own motors in config, so don't let the differential drive do it
        rightMotor.setInverted(true);
        differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

        // turn off the safety so the differentialDrive will stop whining
        differentialDrive.setSafetyEnabled(false);

        this.speed = speed;
        this.turn = turn;
    }

    /**
     * While executing, continually update the differentialDrive with the new speed/turn values
     */
    @Override
    public void execute() {
        super.execute();

        double currentSpeed = speed.getAsDouble();
        double currentTurn = turn.getAsDouble();
        double leftSpeed = currentSpeed + currentTurn;
        double rightSpeed = currentSpeed - currentTurn;
        // every execute, update the differential drive with new args
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

}
