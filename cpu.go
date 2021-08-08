package simplenec

const (
	_ = iota
	modeAbsolute
	modeAbsoluteX
	modeAbsoluteY
	modeAccumulator
	modeImmediate
	modeImplied
	modeIndexedIndirect
	modeIndirect
	modeIndirectIndexed
	modeRelative
	modeZeroPage
	modeZeroPageX
	modeZeroPageY
)

// interrupt types
const (
	_ = iota
	interruptNone
	interruptNMI
	interruptIRQ
)

// instructionModes indicates the addressing mode for each instruction
var instructionModes = [256]byte{
	6, 7, 6, 7, 11, 11, 11, 11, 6, 5, 4, 5, 1, 1, 1, 1,
	10, 9, 6, 9, 12, 12, 12, 12, 6, 3, 6, 3, 2, 2, 2, 2,
	1, 7, 6, 7, 11, 11, 11, 11, 6, 5, 4, 5, 1, 1, 1, 1,
	10, 9, 6, 9, 12, 12, 12, 12, 6, 3, 6, 3, 2, 2, 2, 2,
	6, 7, 6, 7, 11, 11, 11, 11, 6, 5, 4, 5, 1, 1, 1, 1,
	10, 9, 6, 9, 12, 12, 12, 12, 6, 3, 6, 3, 2, 2, 2, 2,
	6, 7, 6, 7, 11, 11, 11, 11, 6, 5, 4, 5, 8, 1, 1, 1,
	10, 9, 6, 9, 12, 12, 12, 12, 6, 3, 6, 3, 2, 2, 2, 2,
	5, 7, 5, 7, 11, 11, 11, 11, 6, 5, 6, 5, 1, 1, 1, 1,
	10, 9, 6, 9, 12, 12, 13, 13, 6, 3, 6, 3, 2, 2, 3, 3,
	5, 7, 5, 7, 11, 11, 11, 11, 6, 5, 6, 5, 1, 1, 1, 1,
	10, 9, 6, 9, 12, 12, 13, 13, 6, 3, 6, 3, 2, 2, 3, 3,
	5, 7, 5, 7, 11, 11, 11, 11, 6, 5, 6, 5, 1, 1, 1, 1,
	10, 9, 6, 9, 12, 12, 12, 12, 6, 3, 6, 3, 2, 2, 2, 2,
	5, 7, 5, 7, 11, 11, 11, 11, 6, 5, 6, 5, 1, 1, 1, 1,
	10, 9, 6, 9, 12, 12, 12, 12, 6, 3, 6, 3, 2, 2, 2, 2,
}

// instructionSizes indicates the size of each instruction in bytes
var instructionSizes = [256]byte{
	2, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	3, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	1, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	1, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 0, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 0, 3, 0, 0,
	2, 2, 2, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
}

// instructionCycles indicates the number of cycles used by each instruction,
// not including conditional cycles
var instructionCycles = [256]byte{
	7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
	2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
	2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
	2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4,
	2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
}

// instructionPageCycles indicates the number of cycles used by each
// instruction when a page is crossed
var instructionPageCycles = [256]byte{
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
}

// http://www.obelisk.me.uk/6502/reference.html
var instuctionNames = [256]string{
	"BRK", "ORA", "KIL", "SLO", "NOP", "ORA", "ASL", "SLO",
	"PHP", "ORA", "ASL", "ANC", "NOP", "ORA", "ASL", "SLO",
	"BPL", "ORA", "KIL", "SLO", "NOP", "ORA", "ASL", "SLO",
	"CLC", "ORA", "NOP", "SLO", "NOP", "ORA", "ASL", "SLO",
	"JSR", "AND", "KIL", "RLA", "BIT", "AND", "ROL", "RLA",
	"PLP", "AND", "ROL", "ANC", "BIT", "AND", "ROL", "RLA",
	"BMI", "AND", "KIL", "RLA", "NOP", "AND", "ROL", "RLA",
	"SEC", "AND", "NOP", "RLA", "NOP", "AND", "ROL", "RLA",
	"RTI", "EOR", "KIL", "SRE", "NOP", "EOR", "LSR", "SRE",
	"PHA", "EOR", "LSR", "ALR", "JMP", "EOR", "LSR", "SRE",
	"BVC", "EOR", "KIL", "SRE", "NOP", "EOR", "LSR", "SRE",
	"CLI", "EOR", "NOP", "SRE", "NOP", "EOR", "LSR", "SRE",
	"RTS", "ADC", "KIL", "RRA", "NOP", "ADC", "ROR", "RRA",
	"PLA", "ADC", "ROR", "ARR", "JMP", "ADC", "ROR", "RRA",
	"BVS", "ADC", "KIL", "RRA", "NOP", "ADC", "ROR", "RRA",
	"SEI", "ADC", "NOP", "RRA", "NOP", "ADC", "ROR", "RRA",
	"NOP", "STA", "NOP", "SAX", "STY", "STA", "STX", "SAX",
	"DEY", "NOP", "TXA", "XAA", "STY", "STA", "STX", "SAX",
	"BCC", "STA", "KIL", "AHX", "STY", "STA", "STX", "SAX",
	"TYA", "STA", "TXS", "TAS", "SHY", "STA", "SHX", "AHX",
	"LDY", "LDA", "LDX", "LAX", "LDY", "LDA", "LDX", "LAX",
	"TAY", "LDA", "TAX", "LAX", "LDY", "LDA", "LDX", "LAX",
	"BCS", "LDA", "KIL", "LAX", "LDY", "LDA", "LDX", "LAX",
	"CLV", "LDA", "TSX", "LAS", "LDY", "LDA", "LDX", "LAX",
	"CPY", "CMP", "NOP", "DCP", "CPY", "CMP", "DEC", "DCP",
	"INY", "CMP", "DEX", "AXS", "CPY", "CMP", "DEC", "DCP",
	"BNE", "CMP", "KIL", "DCP", "NOP", "CMP", "DEC", "DCP",
	"CLD", "CMP", "NOP", "DCP", "NOP", "CMP", "DEC", "DCP",
	"CPX", "SBC", "NOP", "ISC", "CPX", "SBC", "INC", "ISC",
	"INX", "SBC", "NOP", "SBC", "CPX", "SBC", "INC", "ISC",
	"BEQ", "SBC", "KIL", "ISC", "NOP", "SBC", "INC", "ISC",
	"SED", "SBC", "NOP", "ISC", "NOP", "SBC", "INC", "ISC",
}

var insTables = [256]func(*CPU, *runStep){
	brk, ora, kil, slo, nop, ora, asl, slo,
	php, ora, asl, anc, nop, ora, asl, slo,
	bpl, ora, kil, slo, nop, ora, asl, slo,
	clc, ora, nop, slo, nop, ora, asl, slo,
	jsr, and, kil, rla, bit, and, rol, rla,
	plp, and, rol, anc, bit, and, rol, rla,
	bmi, and, kil, rla, nop, and, rol, rla,
	sec, and, nop, rla, nop, and, rol, rla,
	rti, eor, kil, sre, nop, eor, lsr, sre,
	pha, eor, lsr, alr, jmp, eor, lsr, sre,
	bvc, eor, kil, sre, nop, eor, lsr, sre,
	cli, eor, nop, sre, nop, eor, lsr, sre,
	rts, adc, kil, rra, nop, adc, ror, rra,
	pla, adc, ror, arr, jmp, adc, ror, rra,
	bvs, adc, kil, rra, nop, adc, ror, rra,
	sei, adc, nop, rra, nop, adc, ror, rra,
	nop, sta, nop, sax, sty, sta, stx, sax,
	dey, nop, txa, xaa, sty, sta, stx, sax,
	bcc, sta, kil, ahx, sty, sta, stx, sax,
	tya, sta, txs, tas, shy, sta, shx, ahx,
	ldy, lda, ldx, lax, ldy, lda, ldx, lax,
	tay, lda, tax, lax, ldy, lda, ldx, lax,
	bcs, lda, kil, lax, ldy, lda, ldx, lax,
	clv, lda, tsx, las, ldy, lda, ldx, lax,
	cpy, cmp, nop, dcp, cpy, cmp, dec, dcp,
	iny, cmp, dex, axs, cpy, cmp, dec, dcp,
	bne, cmp, kil, dcp, nop, cmp, dec, dcp,
	cld, cmp, nop, dcp, nop, cmp, dec, dcp,
	cpx, sbc, nop, isc, cpx, sbc, inc, isc,
	inx, sbc, nop, sbc, cpx, sbc, inc, isc,
	beq, sbc, kil, isc, nop, sbc, inc, isc,
	sed, sbc, nop, isc, nop, sbc, inc, isc,
}

type runStep struct {
	address uint16 // address of program
	pc      uint16 // Program Counter (PC)
	mode    byte
}

type StatusRegister struct {
	c byte // Carry Flag
	z byte // Zero Flag
	i byte // Interrupt Disable
	d byte // Decimal Mode Flag
	b byte // Break Command
	u byte //
	v byte // Overflow Flag
	n byte // Negative Flag
}

type CPU struct {
	StatusRegister
	memory
	pc        uint16 // Program Counter (PC)
	sp        byte   // Stack Pointer Memory space [0x0100 .. 0x1FF] is used for stack
	a         byte   // Accumulator (A)
	x         byte   // Index Register X (X)
	y         byte   // Index Register Y (Y)
	p         byte   // Processor status (P)
	interrupt byte   // interrupt type to perform
	stall     int    // number of cycles to stall
	cycles    uint64 // number of cycles
}

// setZ sets the zero flag if the argument is zero
func (sr *StatusRegister) setZ(val byte) {
	if val == 0 {
		sr.z = 1
	} else {
		sr.z = 0
	}
}

// getFlags gets the processor status flags
func (sr *StatusRegister) getFlags() byte {
	var flags byte
	flags |= sr.c << 0
	flags |= sr.z << 1
	flags |= sr.i << 2
	flags |= sr.d << 3
	flags |= sr.b << 4
	flags |= sr.u << 5
	flags |= sr.v << 6
	flags |= sr.n << 7
	return flags
}

// setFlags sets the processor status flags
func (sr *StatusRegister) setFlags(flags byte) {
	sr.c = 0x1 & flags
	sr.z = 0x1 & (flags >> 1)
	sr.i = 0x1 & (flags >> 2)
	sr.d = 0x1 & (flags >> 3)
	sr.b = 0x1 & (flags >> 4)
	sr.u = 0x1 & (flags >> 5)
	sr.v = 0x1 & (flags >> 6)
	sr.n = 0x1 & (flags >> 7)
}

// setN sets the negative flag if the argument is negative (high bit is set)
func (sr *StatusRegister) setN(val byte) {
	if val&0x80 != 0 {
		sr.n = 1
	} else {
		sr.n = 0
	}
}

// setZN sets the zero flag and the negative flag
func (sr *StatusRegister) setZN(value byte) {
	sr.setZ(value)
	sr.setN(value)
}

func NewCPU() *CPU {
	c := &CPU{}
	return c
}

// NMI - Non-Maskable Interrupt
func (cpu *CPU) nmi() {
	cpu.push16(cpu.pc)
	php(cpu, nil)
	cpu.pc = cpu.read16(0xFFFA)
	cpu.i = 1
	cpu.cycles += 7
}

// IRQ - IRQ Interrupt
func (cpu *CPU) irq() {
	cpu.push16(cpu.pc)
	php(cpu, nil)
	cpu.pc = cpu.read16(0xFFFE)
	cpu.i = 1
	cpu.cycles += 7
}

// read16bug emulates a 6502 bug that caused the low byte to wrap without
// incrementing the high byte
func (cpu *CPU) read16bug(address uint16) uint16 {
	a := address
	b := (a & 0xFF00) | uint16(byte(a)+1)
	lo := cpu.read(a)
	hi := cpu.read(b)
	return uint16(hi)<<8 | uint16(lo)
}

// Step executes a single CPU instruction
func (cpu *CPU) executeStep() int {
	if cpu.stall > 0 {
		cpu.stall--
		return 1
	}

	cycles := cpu.cycles

	switch cpu.interrupt {
	case interruptNMI:
		cpu.nmi()
	case interruptIRQ:
		cpu.irq()
	}
	cpu.interrupt = interruptNone

	opcode := cpu.read(cpu.pc)
	mode := instructionModes[opcode]

	var address uint16
	var pageCrossed bool
	switch mode {
	case modeAbsolute:
		address = cpu.read16(cpu.pc + 1)
	case modeAbsoluteX:
		address = cpu.read16(cpu.pc+1) + uint16(cpu.x)
		pageCrossed = pagesDiffer(address-uint16(cpu.x), address)
	case modeAbsoluteY:
		address = cpu.read16(cpu.pc+1) + uint16(cpu.y)
		pageCrossed = pagesDiffer(address-uint16(cpu.y), address)
	case modeAccumulator:
		address = 0
	case modeImmediate:
		address = cpu.pc + 1
	case modeImplied:
		address = 0
	case modeIndexedIndirect:
		address = cpu.read16bug(uint16(cpu.read(cpu.pc+1) + cpu.x))
	case modeIndirect:
		address = cpu.read16bug(cpu.read16(cpu.pc + 1))
	case modeIndirectIndexed:
		address = cpu.read16bug(uint16(cpu.read(cpu.pc+1))) + uint16(cpu.y)
		pageCrossed = pagesDiffer(address-uint16(cpu.y), address)
	case modeRelative:
		offset := uint16(cpu.read(cpu.pc + 1))
		if offset < 0x80 {
			address = cpu.pc + 2 + offset
		} else {
			address = cpu.pc + 2 + offset - 0x100
		}
	case modeZeroPage:
		address = uint16(cpu.read(cpu.pc + 1))
	case modeZeroPageX:
		address = uint16(cpu.read(cpu.pc+1)+cpu.x) & 0xff
	case modeZeroPageY:
		address = uint16(cpu.read(cpu.pc+1)+cpu.y) & 0xff
	}

	cpu.pc += uint16(instructionSizes[opcode])
	cpu.cycles += uint64(instructionCycles[opcode])
	if pageCrossed {
		cpu.cycles += uint64(instructionPageCycles[opcode])
	}
	info := &runStep{address, cpu.pc, mode}
	insTables[opcode](cpu, info)

	return int(cpu.cycles - cycles)
}

// ADC - Add with Carry
func adc(cpu *CPU, step *runStep) {
	a := cpu.a
	b := cpu.read(step.address)
	c := cpu.c
	cpu.a = a + b + c
	cpu.setZN(cpu.a)
	if int(a)+int(b)+int(c) > 0xFF {
		cpu.c = 1
	} else {
		cpu.c = 0
	}
	if (a^b)&0x80 == 0 && (a^cpu.a)&0x80 != 0 {
		cpu.v = 1
	} else {
		cpu.v = 0
	}
}

// BRK - Force Interrupt
func brk(cpu *CPU, step *runStep) {
	cpu.push16(cpu.pc)
	php(cpu, step)
	sei(cpu, step)
	cpu.pc = cpu.read16(0xFFFE)
}

// push16 pushes two bytes onto the stack
func (cpu *CPU) push16(value uint16) {
	hi := byte(value >> 8)
	lo := byte(value & 0xFF)
	cpu.push(hi)
	cpu.push(lo)
}

// push pushes a byte onto the stack
func (cpu *CPU) push(value byte) {
	cpu.write(0x100|uint16(cpu.sp), value)
	cpu.sp--
}

// pull16 pops two bytes from the stack
func (cpu *CPU) pull16() uint16 {
	lo := uint16(cpu.pull())
	hi := uint16(cpu.pull())
	return hi<<8 | lo
}

// pull pops a byte from the stack
func (cpu *CPU) pull() byte {
	cpu.sp++
	return cpu.read(0x100 | uint16(cpu.sp))
}

// Read16 reads two bytes using Read to return a double-word value
func (cpu *CPU) read16(address uint16) uint16 {
	lo := uint16(cpu.read(address))
	hi := uint16(cpu.read(address + 1))
	return hi<<8 | lo
}

// pagesDiffer returns true if the two addresses reference different pages
func pagesDiffer(a, b uint16) bool {
	return a&0xFF00 != b&0xFF00
}

// addBranchCycles adds a cycle for taking a branch and adds another cycle
// if the branch jumps to a new page
func (cpu *CPU) addBranchCycles(step *runStep) {
	cpu.cycles++
	if pagesDiffer(step.pc, step.address) {
		cpu.cycles++
	}
}

func (cpu *CPU) compare(a, b byte) {
	cpu.setZN(a - b)
	if a >= b {
		cpu.c = 1
	} else {
		cpu.c = 0
	}
}

// BPL - Branch if Positive
func bpl(cpu *CPU, step *runStep) {
	if cpu.n == 0 {
		cpu.pc = step.address
		cpu.addBranchCycles(step)
	}
}

// BNE - Branch if Not Equal
func bne(cpu *CPU, step *runStep) {
	if cpu.z == 0 {
		cpu.pc = step.address
		cpu.addBranchCycles(step)
	}
}

// CLC - Clear Carry Flag
func clc(cpu *CPU, step *runStep) {
	cpu.c = 0
}

// CLD - Clear Decimal Mode
func cld(cpu *CPU, step *runStep) {
	cpu.d = 0
}

// JSR - Jump to Subroutine
func jsr(cpu *CPU, step *runStep) {
	cpu.push16(cpu.pc - 1)
	cpu.pc = step.address
}

// AND - Logical AND
func and(cpu *CPU, step *runStep) {
	cpu.a = cpu.a & cpu.read(step.address)
	cpu.setZN(cpu.a)
}

// BIT - Bit Test
func bit(cpu *CPU, step *runStep) {
	value := cpu.read(step.address)
	cpu.v = (value >> 6) & 1
	cpu.setZ(value & cpu.a)
	cpu.setN(value)
}

// BMI - Branch if Minus
func bmi(cpu *CPU, step *runStep) {
	if cpu.n != 0 {
		cpu.pc = step.address
		cpu.addBranchCycles(step)
	}
}

// EOR - Exclusive OR
func eor(cpu *CPU, step *runStep) {
	cpu.a = cpu.a ^ cpu.read(step.address)
	cpu.setZN(cpu.a)
}

// LSR - Logical Shift Right
func lsr(cpu *CPU, step *runStep) {
	if step.mode == modeAccumulator {
		cpu.c = cpu.a & 1
		cpu.a >>= 1
		cpu.setZN(cpu.a)
	} else {
		value := cpu.read(step.address)
		cpu.c = value & 1
		value >>= 1
		cpu.write(step.address, value)
		cpu.setZN(value)
	}
}

// JMP - Jump
func jmp(cpu *CPU, step *runStep) {
	cpu.pc = step.address
}

// BVC - Branch if Overflow Clear
func bvc(cpu *CPU, step *runStep) {
	if cpu.v == 0 {
		cpu.pc = step.address
		cpu.addBranchCycles(step)
	}
}

// BVS - Branch if Overflow Set
func bvs(cpu *CPU, step *runStep) {
	if cpu.v != 0 {
		cpu.pc = step.address
		cpu.addBranchCycles(step)
	}
}

// CLI - Clear Interrupt Disable
func cli(cpu *CPU, step *runStep) {
	cpu.i = 0
}

// DEY - Decrement Y Register
func dey(cpu *CPU, step *runStep) {
	cpu.y--
	cpu.setZN(cpu.y)
}

// BCC - Branch if Carry Clear
func bcc(cpu *CPU, step *runStep) {
	if cpu.c == 0 {
		cpu.pc = step.address
		cpu.addBranchCycles(step)
	}
}

// BCS - Branch if Carry Set
func bcs(cpu *CPU, step *runStep) {
	if cpu.c != 0 {
		cpu.pc = step.address
		cpu.addBranchCycles(step)
	}
}

// BEQ - Branch if Equal
func beq(cpu *CPU, step *runStep) {
	if cpu.z != 0 {
		cpu.pc = step.address
		cpu.addBranchCycles(step)
	}
}

// LDA - Load Accumulator
func lda(cpu *CPU, step *runStep) {
	cpu.a = cpu.read(step.address)
	cpu.setZN(cpu.a)
}

// LDX - Load X Register
func ldx(cpu *CPU, step *runStep) {
	cpu.x = cpu.read(step.address)
	cpu.setZN(cpu.x)
}

// LDY - Load Y Register
func ldy(cpu *CPU, step *runStep) {
	cpu.y = cpu.read(step.address)
	cpu.setZN(cpu.y)
}

// CLV - Clear Overflow Flag
func clv(cpu *CPU, step *runStep) {
	cpu.v = 0
}

// CMP - Compare
func cmp(cpu *CPU, step *runStep) {
	value := cpu.read(step.address)
	cpu.compare(cpu.a, value)
}

// CPX - Compare X Register
func cpx(cpu *CPU, step *runStep) {
	value := cpu.read(step.address)
	cpu.compare(cpu.x, value)
}

// CPY - Compare Y Register
func cpy(cpu *CPU, step *runStep) {
	value := cpu.read(step.address)
	cpu.compare(cpu.y, value)
}

// DEC - Decrement Memory
func dec(cpu *CPU, step *runStep) {
	value := cpu.read(step.address) - 1
	cpu.write(step.address, value)
	cpu.setZN(value)
}

// DEX - Decrement X Register
func dex(cpu *CPU, step *runStep) {
	cpu.x--
	cpu.setZN(cpu.x)
}

// ORA - Logical Inclusive OR
func ora(cpu *CPU, step *runStep) {
	cpu.a = cpu.a | cpu.read(step.address)
	cpu.setZN(cpu.a)
}

// PHA - Push Accumulator
func pha(cpu *CPU, step *runStep) {
	cpu.push(cpu.a)
}

// PHP - Push Processor Status
func php(cpu *CPU, step *runStep) {
	cpu.push(cpu.getFlags() | 0x10)
}

// PLA - Pull Accumulator
func pla(cpu *CPU, step *runStep) {
	cpu.a = cpu.pull()
	cpu.setZN(cpu.a)
}

// PLP - Pull Processor Status
func plp(cpu *CPU, step *runStep) {
	cpu.setFlags(cpu.pull()&0xEF | 0x20)
}

// ROL - Rotate Left
func rol(cpu *CPU, step *runStep) {
	if step.mode == modeAccumulator {
		c := cpu.c
		cpu.c = (cpu.a >> 7) & 1
		cpu.a = (cpu.a << 1) | c
		cpu.setZN(cpu.a)
	} else {
		c := cpu.a
		value := cpu.read(step.address)
		cpu.c = (value >> 7) & 1
		value = (value << 1) | c
		cpu.write(step.address, value)
		cpu.setZN(value)
	}
}

// ROR - Rotate Right
func ror(cpu *CPU, step *runStep) {
	if step.mode == modeAccumulator {
		c := cpu.c
		cpu.c = cpu.a & 1
		cpu.a = (cpu.a >> 1) | (c << 7)
		cpu.setZN(cpu.a)
	} else {
		c := cpu.c
		value := cpu.read(step.address)
		cpu.c = value & 1
		value = (value >> 1) | (c << 7)
		cpu.write(step.address, value)
		cpu.setZN(value)
	}
}

// RTI - Return from Interrupt
func rti(cpu *CPU, step *runStep) {
	cpu.setFlags(cpu.pull()&0xEF | 0x20)
	cpu.pc = cpu.pull16()
}

// RTS - Return from Subroutine
func rts(cpu *CPU, step *runStep) {
	cpu.pc = cpu.pull16() + 1
}

// SBC - Subtract with Carry
func sbc(cpu *CPU, step *runStep) {
	a := cpu.a
	b := cpu.read(step.address)
	c := cpu.c
	cpu.a = a - b - (1 - c)
	cpu.setZN(cpu.a)
	if int(a)-int(b)-int(1-c) >= 0 {
		cpu.c = 1
	} else {
		cpu.c = 0
	}
	if (a^b)&0x80 != 0 && (a^cpu.a)&0x80 != 0 {
		cpu.v = 1
	} else {
		cpu.v = 0
	}
}

// SEC - Set Carry Flag
func sec(cpu *CPU, step *runStep) {
	cpu.c = 1
}

// SED - Set Decimal Flag
func sed(cpu *CPU, step *runStep) {
	cpu.d = 1
}

// SEI - Set Interrupt Disable
func sei(cpu *CPU, step *runStep) {
	cpu.i = 1
}

// STA - Store Accumulator
func sta(cpu *CPU, step *runStep) {
	cpu.write(step.address, cpu.a)
}

// STX - Store X Register
func stx(cpu *CPU, step *runStep) {
	cpu.write(step.address, cpu.x)
}

// STY - Store Y Register
func sty(cpu *CPU, step *runStep) {
	cpu.write(step.address, cpu.y)
}

// TAX - Transfer Accumulator to X
func tax(cpu *CPU, step *runStep) {
	cpu.x = cpu.a
	cpu.setZN(cpu.x)
}

// TAY - Transfer Accumulator to Y
func tay(cpu *CPU, step *runStep) {
	cpu.y = cpu.a
	cpu.setZN(cpu.y)
}

// TSX - Transfer Stack Pointer to X
func tsx(cpu *CPU, step *runStep) {
	cpu.x = cpu.sp
	cpu.setZN(cpu.x)
}

// TXA - Transfer X to Accumulator
func txa(cpu *CPU, step *runStep) {
	cpu.a = cpu.x
	cpu.setZN(cpu.a)
}

// TXS - Transfer X to Stack Pointer
func txs(cpu *CPU, step *runStep) {
	cpu.sp = cpu.x
}

// TYA - Transfer Y to Accumulator
func tya(cpu *CPU, step *runStep) {
	cpu.a = cpu.y
	cpu.setZN(cpu.a)
}

// INC - Increment Memory
func inc(cpu *CPU, step *runStep) {
	value := cpu.read(step.address) + 1
	cpu.write(step.address, value)
	cpu.setZN(value)
}

// INY - Increment Y Register
func iny(cpu *CPU, step *runStep) {
	cpu.y++
	cpu.setZN(cpu.y)
}

// INX - Increment X Register
func inx(cpu *CPU, step *runStep) {
	cpu.x++
	cpu.setZN(cpu.x)
}

// ASL - Arithmetic Shift Left
func asl(cpu *CPU, step *runStep) {
	if step.mode == modeAccumulator {
		cpu.c = (cpu.a >> 7) & 1
		cpu.a <<= 1
		cpu.setZN(cpu.a)
	} else {
		value := cpu.read(step.address)
		cpu.c = (value >> 7) & 1
		value <<= 1
		cpu.write(step.address, value)
		cpu.setZN(value)
	}
}

func ahx(cpu *CPU, step *runStep) {
}

func alr(cpu *CPU, step *runStep) {
}

func anc(cpu *CPU, step *runStep) {
}

func arr(cpu *CPU, step *runStep) {
}

func axs(cpu *CPU, step *runStep) {
}

func dcp(cpu *CPU, step *runStep) {
}

func isc(cpu *CPU, step *runStep) {
}

func kil(cpu *CPU, step *runStep) {
}

func las(cpu *CPU, step *runStep) {
}

func lax(cpu *CPU, step *runStep) {
}

func rla(cpu *CPU, step *runStep) {
}

func rra(cpu *CPU, step *runStep) {
}

func sax(cpu *CPU, step *runStep) {
}

func shx(cpu *CPU, step *runStep) {
}

func shy(cpu *CPU, step *runStep) {
}

func slo(cpu *CPU, step *runStep) {
}

func sre(cpu *CPU, step *runStep) {
}

func tas(cpu *CPU, step *runStep) {
}

func xaa(cpu *CPU, step *runStep) {
}

func nop(cpu *CPU, step *runStep) {
}
