package frc.robot.commands.shooterindexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.ShooterIndexer;

public class ShooterIndexerFullSpeed extends Command
{
    public ShooterIndexerFullSpeed ()
    {
        addRequirements(ShooterIndexer.getInstance());
        //addRequirements(Indexer.getInstance());
    }

    @Override
    public void initialize()
    {
        ShooterIndexer.getInstance().setVoltage(Volts.of(Constants.ShooterIndexer.INTAKE_VOLTAGE));
        /*Indexer.getInstance().setMainVoltage(Volts.of(Constants.Indexer.MAIN_MAX_VOLTAGE));
        Indexer.getInstance().setSideVoltage(Volts.of(Constants.Indexer.SIDE_MAX_VOLTAGE));*/
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        ShooterIndexer.getInstance().setVoltage(Volts.of(0.0));
    }
    
}
