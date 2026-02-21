package frc.robot.util;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.struct.Struct;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Simulation;
import frc.robot.simulation.SimulationState;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class TelemetryUtil {
    static NetworkTableInstance netTableInstance = NetworkTableInstance.getDefault();
    static NetworkTable table = netTableInstance.getTable("telemetry");

    
    static List<DoublePublisher> doublePublishers =  new ArrayList<>();
    static List<DoubleSupplier> doublePublisherSuppliers = new ArrayList<>();

    static List<BooleanPublisher> booleanPublishers =  new ArrayList<>();
    static List<BooleanSupplier> booleanPublisherSuppliers = new ArrayList<>();

    static List<StringPublisher> stringPublishers = new ArrayList<>();
    static List<Supplier<String>> stringPublisherSuppliers = new ArrayList<>();

    static List<IntegerPublisher> integerPublishers = new ArrayList<>();
    static List<IntSupplier> integerPublisherSuppliers = new ArrayList<>();

    static List<StructPublisher<?>> structPublishers = new ArrayList<>();
    static List<Supplier<?>> structPublisherSuppliers = new ArrayList<>();

    static List<StructArrayPublisher<?>> structArrayPublishers = new ArrayList<>();
    static List<Supplier<?>> structArrayPublisherSuppliers = new ArrayList<>();


    public static void addDouble(String name, DoubleSupplier supplier)
    {
        doublePublishers.add(table.getDoubleTopic(name).publish());
        doublePublisherSuppliers.add(supplier);
    }

    public static void addBoolean(String name, BooleanSupplier supplier)
    {
        booleanPublishers.add(table.getBooleanTopic(name).publish());
        booleanPublisherSuppliers.add(supplier);
    }
    
    public static void addString(String name, Supplier<String> supplier) 
    { 
        stringPublishers.add(table.getStringTopic(name).publish());
        stringPublisherSuppliers.add(supplier);
    }

    public static void addInteger(String name, IntSupplier supplier) 
    { 
        integerPublishers.add(table.getIntegerTopic(name).publish());
        integerPublisherSuppliers.add(supplier);
    }

    public static <T> void addStruct(String name, Struct<T> type, Supplier<T> supplier) 
    { 
        structPublishers.add(table.getStructTopic(name, type).publish());
        structPublisherSuppliers.add(supplier);
    }

    public static <T> void addStructArray(String name, Struct<T> type, Supplier<T[]> supplier)
    {
        structArrayPublishers.add(table.getStructArrayTopic(name, type).publish());
        structArrayPublisherSuppliers.add(supplier);
    }


    
    public static void periodic()
    {
        for(int i=0; i < doublePublishers.size(); i++)
        {
            doublePublishers.get(i).set(doublePublisherSuppliers.get(i).getAsDouble());
        }   

        for(int i=0; i < booleanPublishers.size(); i++)
        {
            booleanPublishers.get(i).set(booleanPublisherSuppliers.get(i).getAsBoolean());
        }   

        for(int i=0; i < stringPublishers.size(); i++)
        {
            stringPublishers.get(i).set(stringPublisherSuppliers.get(i).get());
        }   

        for(int i=0; i < integerPublishers.size(); i++)
        {
            integerPublishers.get(i).set(integerPublisherSuppliers.get(i).getAsInt());
        }   

        for (int i = 0; i < structPublishers.size(); i++) 
        {
            ((StructPublisher<Object>) structPublishers.get(i)).set(((Supplier<Object>) structPublisherSuppliers.get(i)).get());
        }

        for (int i = 0; i < structArrayPublishers.size(); i++)
        {
            ((StructArrayPublisher<Object>) structArrayPublishers.get(i)).set((Object[]) structArrayPublisherSuppliers.get(i).get());
        }

    }
}