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
	hz         float64
}

func SpreadCycle() []Register {
	return []Register{
		&NodeConf{SendDelay: 2},
		&Gconf{EnSpreadcycle: 1, PdnDisable: 1, MstepRegSelect: 1, IndexStep: 1},
		&IholdIrun{Ihold: 1, Irun: 31},
		&PWMConf{PwmAutoscale: 1, PwmAutograd: 1, PwmReg: 8},
	}
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
	return m.write(&cf)
}

func (m *Motor) Setup(opts ...Register) error {
	ifcnt, err := m.read(IFCNT)
	if err != nil {
		return err
	}

	for _, opt := range opts {
		if err := m.write(opt); err != nil {
			return err
		}
	}

	if err := m.Microsteps(m.microsteps); err != nil {
		return err
	}

	ifcnt2, err := m.read(IFCNT)
	if err != nil {
		return err
	}

	if ifcnt2-ifcnt != uint32(len(opts)+1) {
		return fmt.Errorf("setup failed, %d writes not registered (%d)", len(opts)+1, ifcnt2-ifcnt)
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
func (m *Motor) Move(hz float64) error {
	for _, r := range m.ramp(hz) {
		if err := m.write(&Vactual{Velocity: uint32((r / 0.715) * 200 * float64(m.microsteps))}); err != nil {
			return err
		}
		time.Sleep(50 * time.Millisecond)
	}
	m.hz = hz
	return nil
}

func (m *Motor) Ifcnt(rps float64) (uint32, error) {
	return m.read(IFCNT)
}

func (m *Motor) write(r Register) error {
	register, value := r.Pack()
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

func (m *Motor) ramp(hz float64) []float64 {
	var ramp []float64

	if hz < 0 {
		return ramp
	}

	if hz > m.hz {
		for i := m.hz + 1; i < hz; i++ {
			if i >= hz {
				i = hz
			}
			ramp = append(ramp, i)
		}
		ramp = append(ramp, hz)
	} else {
		for i := m.hz - 1; i > hz-1; i-- {
			if i <= hz {
				i = hz
			}
			if i > 0 {
				ramp = append(ramp, i)
			}
		}
		ramp = append(ramp, hz)
	}

	return ramp
}
