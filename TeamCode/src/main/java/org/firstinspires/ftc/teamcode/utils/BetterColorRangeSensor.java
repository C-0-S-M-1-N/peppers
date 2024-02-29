package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
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
                REGISTERS.MAIN_STATUS.reg,
                1,
                mode
        ));
    }

    protected short readProximity(REGISTERS r){
        return TypeConversion.byteArrayToShort(device.read(r.reg, 2), ByteOrder.LITTLE_ENDIAN);
    }
    protected byte readRawByte(REGISTERS r){
        return device.read8(r.reg);
    }
    protected void writeToDevice(final REGISTERS r, short val){
        device.write(r.reg, TypeConversion.shortToByteArray(val, ByteOrder.LITTLE_ENDIAN));
    }
    protected short readByte(REGISTERS r){
        return TypeConversion.byteArrayToShort(device.read(r.reg, 1), ByteOrder.LITTLE_ENDIAN);
    }

    /**
     * This is a very expensive operation DO NOT USE IT OUTSIDE OF INIT!
     *
     *
     * default value = 0
     *
     * @param val the threshold value for close signal to be triggered
     * */
    public void setThresHold(int val){
        byte[] send = {(byte) (val & 0xFF), (byte) (val & 0x0300)};
        device.write8(REGISTERS.PS_THRES_LOW_0.reg ,send[0], I2cWaitControl.NONE);
        device.write8(REGISTERS.PS_THRES_LOW_1.reg ,send[1], I2cWaitControl.NONE);
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
    public boolean LogicProximityStatus(){
        return (readRawByte(REGISTERS.MAIN_STATUS) & 0x4) != 0;
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

        PS_THRES_LOW_0(0x1D),
        PS_THRES_LOW_1(0x1E);

        REGISTERS(int hex){
           reg = hex;
        }
        int reg = 0x0;
    }

}