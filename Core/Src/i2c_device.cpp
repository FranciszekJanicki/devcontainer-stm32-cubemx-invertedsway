#include "i2c_device.hpp"

using namespace Utility;

namespace InvertedSway {

    I2CDevice::I2CDevice(I2CBusHandle const i2c_bus, std::uint16_t const device_address) noexcept :
        i2c_bus_{i2c_bus}, device_address_{device_address}
    {
        this->initialize();
    }

    void I2CDevice::initialize() noexcept
    {
        if (this->i2c_bus_ != nullptr) {
            if (HAL_I2C_IsDeviceReady(this->i2c_bus_, this->device_address_ << 1, I2C_SCAN_RETRIES, I2C_TIMEOUT) ==
                HAL_OK) {
                this->initialized_ = true;
            }
        }
    }

    DWord I2CDevice::read_dword(std::uint8_t const reg_address) const noexcept
    {
        return this->read_dwords<1>(reg_address)[0];
    }

    Word I2CDevice::read_word(std::uint8_t const reg_address) const noexcept
    {
        return this->read_words<1>(reg_address)[0];
    }

    void I2CDevice::read_bytes(std::uint8_t const reg_address,
                               Byte* const read_data,
                               std::size_t const read_size) const noexcept
    {
        if (this->initialized_) {
            HAL_I2C_Mem_Read(this->i2c_bus_,
                             this->device_address_ << 1,
                             reg_address,
                             sizeof(reg_address),
                             read_data,
                             read_size,
                             I2C_TIMEOUT);
        }
    }

    Byte I2CDevice::read_byte(std::uint8_t const reg_address) const noexcept
    {
        return this->read_bytes<1>(reg_address)[0];
    }

    Bit I2CDevice::read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept
    {
        return this->read_byte(reg_address) & (1 << read_position);
    }

    Byte I2CDevice::read_bits(std::uint8_t const reg_address,
                              std::uint8_t const read_position,
                              std::size_t const read_size) const noexcept
    {
        Byte read = this->read_byte(reg_address);
        Byte mask = ((1 << read_size) - 1) << (read_position - read_size + 1);
        read &= mask;
        read >>= (read_position - read_size + 1);
        return read;
    }

    void I2CDevice::write_dword(std::uint8_t const reg_address, DWord const write_data) const noexcept
    {
        this->write_dwords(reg_address, DWords<1>{write_data});
    }

    void I2CDevice::write_word(std::uint8_t const reg_address, Word const write_data) const noexcept
    {
        this->write_words(reg_address, Words<1>{write_data});
    }

    void I2CDevice::write_bytes(std::uint8_t const reg_address,
                                Byte* const write_data,
                                std::size_t const write_size) const noexcept
    {
        if (this->initialized_) {
            HAL_I2C_Mem_Write(this->i2c_bus_,
                              this->device_address_ << 1,
                              reg_address,
                              sizeof(reg_address),
                              write_data,
                              write_size,
                              I2C_TIMEOUT);
        }
    }

    void I2CDevice::write_byte(std::uint8_t const reg_address, Byte const write_data) const noexcept
    {
        this->write_bytes(reg_address, Bytes<1>{write_data});
    }

    void I2CDevice::write_bit(std::uint8_t const reg_address,
                              Bit const write_data,
                              std::uint8_t const write_position) const noexcept
    {
        Byte write = this->read_byte(reg_address);
        if (write_data) {
            write |= (1 << write_position);
        } else {
            write &= ~(1 << write_position);
        }
        this->write_byte(reg_address, write);
    }

    void I2CDevice::write_bits(std::uint8_t const reg_address,
                               Byte const write_data,
                               std::uint8_t const write_position,
                               std::size_t const write_size) const noexcept
    {
        Byte write = this->read_byte(reg_address);
        Byte mask = ((1 << write_size) - 1) << (write_position - write_size + 1);
        Byte temp = write_data << (write_position - write_size + 1);
        temp &= mask;
        write &= ~(mask);
        write |= temp;
        this->write_byte(reg_address, write);
    }

}; // namespace InvertedSway