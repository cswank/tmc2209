package tmc2209

import (
	"fmt"
	"time"

	"go.bug.st/serial"
)

type Motor struct {
	uart       serial.Port
	address    uint8
	microsteps uint32
}

func New(uart serial.Port, address uint8, ms uint32) *Motor {
	return &Motor{
		uart:       uart,
		address:    address,
		microsteps: ms,
	}
}

func (m *Motor) Microsteps(ms uint32) error {
	m.microsteps = ms
	cf := Chopconf{
		Toff: 5,
		Mres: mres(ms),
	}
	return m.write(CHOPCONF, cf.Pack())
}

func (m *Motor) Setup() error {
	x, err := m.read(GSTAT)
	fmt.Printf("gstat: %03b, err: %v\n", x, err)

	x, err = m.read(IFCNT)
	fmt.Printf("ifcnt: %d, err: %v\n", x, err)

	nc := NodeConf{SendDelay: 2}
	if err := m.write(NODECONF, nc.Pack()); err != nil {
		return err
	}

	gc := Gconf{EnSpreadcycle: 1, PdnDisable: 1, MstepRegSelect: 1}
	if err := m.write(GCONF, gc.Pack()); err != nil {
		return err
	}

	fmt.Printf("gconf: %010b\n", gc.Pack())

	x, err = m.read(GCONF)
	fmt.Printf("gconf: %010b, err: %v\n", x, err)

	x, err = m.read(IFCNT)
	fmt.Printf("ifcnt: %d, err: %v\n", x, err)

	if err := m.write(IHOLD_IRUN, 0); err != nil {
		return err
	}

	x, err = m.read(IFCNT)
	fmt.Printf("ifcnt: %d, err: %v\n", x, err)

	pw := PWMConf{PwmAutoscale: 1, PwmAutograd: 1, PwmReg: 8}
	if err := m.write(PWMCONF, pw.Pack()); err != nil {
		return err
	}

	x, err = m.read(IFCNT)
	fmt.Printf("ifcnt: %d, err: %v\n", x, err)

	if err := m.Microsteps(m.microsteps); err != nil {
		return err
	}

	x, err = m.read(GSTAT)
	fmt.Printf("gstat: %b, err: %v\n", x, err)
	return nil
}

// the formula is rps / 0.715 * steps * microsteps
// rps is revolutions per second
// 0.715 is the built in pulse generator
// steps is the number of steps on the motor
// microsteps is the microsteps
// so 1 rev per second on a 200 step motor set to 16 micro steps is
// 1 / 0.715 * 200 * 16 = 4476
func (m *Motor) Move(rps float64) error {
	if err := m.write(VACTUAL, uint32((rps/0.715)*200*float64(m.microsteps))); err != nil {
		return err
	}

	// val, err := m.read(IFCNT)
	// fmt.Printf("IFCNT: %d, err: %v\n", val, err)
	return nil
}

func (m *Motor) write(register uint8, value uint32) error {
	buffer := []byte{
		0x05,                       // Sync byte
		m.address,                  // Slave address
		register | 0x80,            // Write command (set MSB to 1 for write)
		byte((value >> 24) & 0xFF), // MSB of value
		byte((value >> 16) & 0xFF), // Middle byte
		byte((value >> 8) & 0xFF),  // Next byte
		byte(value & 0xFF),         // LSB of value
		0,
	}

	buffer[7] = m.crc(buffer[:7])
	_, err := m.uart.Write(buffer)
	time.Sleep(4 * time.Millisecond)
	return err
}

func (m *Motor) read(register uint8) (uint32, error) {
	m.uart.ResetInputBuffer()

	writeBuffer := []byte{
		0x05,            // Sync byte
		m.address,       // Slave address
		register & 0x7f, // Read command (MSB clear for read)
		0,
	}

	writeBuffer[3] = m.crc(writeBuffer[:3])

	_, err := m.uart.Write(writeBuffer)
	if err != nil {
		return 0, err
	}

	time.Sleep(10 * time.Millisecond)
	trashBuf := make([]byte, 4) //read request write will be in the buffer
	_, err = m.uart.Read(trashBuf)
	if err != nil {
		return 0, err
	}

	time.Sleep(10 * time.Millisecond)

	readBuffer := make([]byte, 8)
	n, err := m.uart.Read(readBuffer)
	if err != nil {
		return 0, err
	}

	if n != 8 {
		return 0, fmt.Errorf("unable to read 8 bytes from motor (%d)", n)
	}

	if !(readBuffer[0] == 0x5 && readBuffer[1] == 0xff) {
		return 0, fmt.Errorf("not a master response (%d)", readBuffer[1])
	}

	checksum := m.crc(readBuffer[:7])
	if checksum != readBuffer[7] {
		return 0, fmt.Errorf("checksum error, expected %x, got %x", checksum, readBuffer[7])
	}

	time.Sleep(10 * time.Millisecond)

	return uint32(readBuffer[3])<<24 |
		uint32(readBuffer[4])<<16 |
		uint32(readBuffer[5])<<8 |
		uint32(readBuffer[6]), nil
}

func (m *Motor) crc(buf []byte) byte {
	crc := byte(0)
	for _, b := range buf {
		for range 8 {
			if (crc>>7)^(b&0x01) == 1 {
				crc = (crc << 1) ^ 0x07
			} else {
				crc = crc << 1
			}
			b = b >> 1
		}
	}
	return crc
}

func mres(microsteps uint32) uint32 {
	var value uint32

	if microsteps == 0 {
		microsteps = 1
	}

	for (microsteps & 0x01) == 0 {
		value++
		microsteps >>= 1
	}

	if value > 8 {
		value = 8
	}

	return 8 - value
}

func Constrain(value, low, high uint32) uint32 {
	if value < low {
		return low
	}
	if value > high {
		return high
	}
	return value
}

func SetRunCurrent(percent uint8) {
	_ = PercentToCurrentSetting(percent)

	// Set the run current register to runCurrent value
}

func SetHoldCurrent(percent uint8) {
	_ = PercentToCurrentSetting(percent)
	// Set the hold current register to holdCurrent value
}
func PercentToCurrentSetting(percent uint8) uint8 {
	constrainedPercent := Constrain(uint32(percent), 0, 100)
	return uint8(Map(constrainedPercent, 0, 100, 0, 255))
}

func CurrentSettingToPercent(currentSetting uint8) uint8 {
	return uint8(Map(uint32(currentSetting), 0, 255, 0, 100))
}

func PercentToHoldDelaySetting(percent uint8) uint8 {
	constrainedPercent := Constrain(uint32(percent), 0, 100)
	return uint8(Map(constrainedPercent, 0, 100, 0, 255))
}

func HoldDelaySettingToPercent(holdDelaySetting uint8) uint8 {
	return uint8(Map(uint32(holdDelaySetting), 0, 255, 0, 100))
}
func Map(value, fromLow, fromHigh, toLow, toHigh uint32) uint32 {
	return (value-fromLow)*(toHigh-toLow)/(fromHigh-fromLow) + toLow
}

// VerifyCommunication checks the communication with the TMC2209 by reading the version register (IOIN).
// It returns true if the communication is successful (i.e., the version matches the expected version).
// VerifyCommunication verifies the communication with the TMC2209 by reading the version register (IOIN).
// It explicitly resets the struct and de-references it after the check to ensure memory is managed manually.
// func VerifyCommunication(comm *Motor, driverIndex uint8) bool {
// 	var io *Ioin
// 	if io == nil {
// 		io = NewIoin() // Initialize the struct if not already initialized
// 	} else {
// 		*io = Ioin{}
// 	}
// 	_, err := comm.read(io.GetAddress())
// 	if err != nil {
// 		return false
// 	}
// 	if io.Version == expectedVersion {
// 		io = nil
// 		return true
// 	}
// 	io = nil
// 	return false
// }

// // CheckErrorStatus verifies the communication and checks for error flags in the TMC2209 driver status.
// // It explicitly resets the struct and de-references it when done to ensure memory is managed manually.
// func CheckErrorStatus(comm RegisterComm, driverIndex uint8) bool {
// 	var d *DrvStatus
// 	if d == nil {
// 		d = NewDrvStatus()
// 	} else {
// 		*d = DrvStatus{}
// 	}
// 	_, err := d.Read(comm, driverIndex)
// 	if err != nil {
// 		return false
// 	}
// 	errorFlags := d.Ola | d.S2vsa | d.S2vsb | d.Ot | d.S2ga | d.S2gb | d.Olb
// 	if errorFlags != 0 {
// 		log.Printf("TMC2209 Error Detected: %X", errorFlags)
// 		return false
// 	}
// 	d = nil
// 	return true
// }

// // GetInterfaceTransmissionCount reads the IFCNT register to check for UART transmission status
// func GetInterfaceTransmissionCount(comm *Motor, driverIndex uint8) (uint32, error) {
// 	ifcnt := NewIfcnt()
// 	_, err := comm.read(ifcnt.GetAddress())
// 	if err != nil {
// 		return 0, err
// 	}
// 	return ifcnt.Bytes, nil
// }
