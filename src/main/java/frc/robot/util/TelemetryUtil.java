
package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;

public class TelemetryUtil 
{
    static NetworkTableInstance netTableInstance = NetworkTableInstance.getDefault();
    static NetworkTable table = netTableInstance.getTable("telemetry");

    static List<DoublePublisher> doublePublishers = new ArrayList<>();
    static List<DoubleSupplier> doublePublisherSuppliers = new ArrayList<>();

    static List<BooleanPublisher> booleanPublishers = new ArrayList<>();
    static List<BooleanSupplier> booleanPublisherSuppliers = new ArrayList<>();

    static List<StringPublisher> stringPublishers = new ArrayList<>();
    static List<Supplier<String>> stringPublisherSuppliers = new ArrayList<>();

    static List<IntegerPublisher> integerPublishers = new ArrayList<>();
    static List<IntSupplier> integerPublisherSuppliers = new ArrayList<>();

    static List<StructPublisher<?>> structPublishers = new ArrayList<>();
    static List<Supplier<?>> structPublisherSuppliers = new ArrayList<>();

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

    public static void periodic() 
    {
        for (int i = 0; i < doublePublishers.size(); i++) 
        {
            doublePublishers.get(i).set(doublePublisherSuppliers.get(i).getAsDouble());
        }

        for (int i = 0; i < booleanPublishers.size(); i++) 
        {
            booleanPublishers.get(i).set(booleanPublisherSuppliers.get(i).getAsBoolean());
        }

        for (int i = 0; i < stringPublishers.size(); i++) 
        {
            stringPublishers.get(i).set(stringPublisherSuppliers.get(i).get());
        }

        for (int i = 0; i < integerPublishers.size(); i++) 
        {
            integerPublishers.get(i).set(integerPublisherSuppliers.get(i).getAsInt());
        }

        for (int i = 0; i < structPublishers.size(); i++) 
        {
            ((StructPublisher<Object>) structPublishers.get(i))
                    .set(((Supplier<Object>) structPublisherSuppliers.get(i)).get());
        }
    }
}