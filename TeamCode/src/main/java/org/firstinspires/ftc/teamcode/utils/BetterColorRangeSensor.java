package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;

@I2cDeviceType
@DeviceProperties(xmlTag = "REVColorSensorv3", name = "REV Color Sensor v3 - proximity")
public class BetterColorRangeSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private I2cDeviceSynch device;
    private I2cDeviceSynch.ReadMode currentMode;

    public void setReadWindow(I2cDeviceSynch.ReadMode mode){
        currentMode = mode;
        this.device.setReadWindow(new I2cDeviceSynch.ReadWindow(
                REGISTERS.PS_DATA_0.reg,
                REGISTERS.PS_DATA_1.reg - REGISTERS.PS_DATA_0.reg + 1,
                mode
        ));
    }

    protected short readProximity(REGISTERS r){
        return TypeConversion.byteArrayToShort(device.read(r.reg, 2), ByteOrder.LITTLE_ENDIAN);
    }
    protected void writeToDevice(final REGISTERS r, short val){
        device.write(r.reg, TypeConversion.shortToByteArray(val, ByteOrder.LITTLE_ENDIAN));
    }
    protected short readByte(REGISTERS r){
        return TypeConversion.byteArrayToShort(device.read(r.reg, 1), ByteOrder.LITTLE_ENDIAN);
    }

    public BetterColorRangeSensor(I2cDeviceSynch device, boolean isOwned){
        super(device, isOwned);

        this.setReadWindow(I2cDeviceSynch.ReadMode.REPEAT);
        this.device.setI2cAddress(I2cAddr.create7bit(0x52));

        super.registerArmingStateCallback(false);
        this.device.engage();
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    public short getManufacturerRawID(){
        return readByte(REGISTERS.PART_ID);
    }

    @Override
    public String getDeviceName() {
        return "REV ColorSensor v3 - proximity";
    }


    public enum REGISTERS{
        MAIN_CTRL(0x0),
        PS_LED(0x1),
        PS_PULSES(0x2),
        PS_MEAS_RATE(0x3),
        LS_MEAS_RATE(0x4),
        LS_Gain(0x5),
        PART_ID(0x6),
        MAIN_STATUS(0x7),
        PS_DATA_0(0x8), // LSB
        PS_DATA_1(0x9), // MSB

        LS_DATA_IR_0(0xA), // LSB
        LS_DATA_IR_1(0xB),
        LS_DATA_IR_2(0xC), // MSB

        LS_DATA_GREEN_0(0xD), // LSB
        LS_DATA_GREEN_1(0xE),
        LS_DATA_GREEN_2(0xF), // MSB

        LS_DATA_BLUE_0(0x10), // LSB
        LS_DATA_BLUE_1(0x11),
        LS_DATA_BLUE_2(0x12), // MSB

        LS_DATA_RED_0(0x13), // LSB
        LS_DATA_RED_1(0x14),
        LS_DATA_RED_2(0x15), // MSB
        ;

        REGISTERS(int hex){
           reg = hex;
        }
        int reg = 0x0;
    }

}
