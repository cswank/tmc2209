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
	ifcnt, err := m.read(IFCNT)
	if err != nil {
		return err
	}

	nc := NodeConf{SendDelay: 2}
	if err := m.write(NODECONF, nc.Pack()); err != nil {
		return err
	}

	gc := Gconf{EnSpreadcycle: 1, PdnDisable: 1, MstepRegSelect: 1}
	if err := m.write(GCONF, gc.Pack()); err != nil {
		return err
	}

	if err := m.write(IHOLD_IRUN, 0); err != nil {
		return err
	}

	pw := PWMConf{PwmAutoscale: 1, PwmAutograd: 1, PwmReg: 8}
	if err := m.write(PWMCONF, pw.Pack()); err != nil {
		return err
	}

	if err := m.Microsteps(m.microsteps); err != nil {
		return err
	}

	ifcnt2, err := m.read(IFCNT)
	if err != nil {
		return err
	}

	if ifcnt2-ifcnt != 5 { //did 5 writes, so make sure they were accepted
		return fmt.Errorf("setup failed, did write 5 times (%d)", ifcnt2-ifcnt)
	}

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
	return m.write(VACTUAL, uint32((rps/0.715)*200*float64(m.microsteps)))
}

func (m *Motor) Ifcnt(rps float64) (uint32, error) {
	return m.read(IFCNT)
}

func (m *Motor) write(register uint8, value uint32) error {
	buffer := []byte{
		0x05,
		m.address,
		register | 0x80,
		byte((value >> 24) & 0xFF),
		byte((value >> 16) & 0xFF),
		byte((value >> 8) & 0xFF),
		byte(value & 0xFF),
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
		0x05,
		m.address,
		register & 0x7f,
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

	time.Sleep(10 * time.Millisecond) //make sure subsequent reads/writes don't happen when the driver's UART is inactive (according to SENDDELAY)

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
