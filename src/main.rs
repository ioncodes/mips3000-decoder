macro_rules! instruction {
    ($name:ident, $opcode_type:ident) => {
        Instruction {
            opcode: Opcode::$name,
            opcode_raw: 0,
            opcode_type: InstructionType::$opcode_type,
        }
    };
    ($name:ident($param:expr), $opcode_type:ident) => {
        Instruction {
            opcode: Opcode::$name($param),
            opcode_raw: 0,
            opcode_type: InstructionType::$opcode_type,
        }
    };
}

// opcode = 0x00
static MIPS_RTYPE_LUT: [Instruction; 64] = [
    /* 0x00 */ instruction!(ShiftLeftLogical, RType),
    /* 0x01 */ instruction!(Invalid, Invalid),
    /* 0x02 */ instruction!(ShiftRightLogical, RType),
    /* 0x03 */ instruction!(ShiftRightArithmetic, RType),
    /* 0x04 */ instruction!(ShiftLeftLogicalVariable, RType),
    /* 0x05 */ instruction!(Invalid, Invalid),
    /* 0x06 */ instruction!(ShiftRightLogicalVariable, RType),
    /* 0x07 */ instruction!(ShiftRightArithmeticVariable, RType),
    /* 0x08 */ instruction!(JumpRegister, RType),
    /* 0x09 */ instruction!(JumpAndLinkRegister, RType),
    /* 0x0A */ instruction!(Invalid, Invalid),
    /* 0x0B */ instruction!(Invalid, Invalid),
    /* 0x0C */ instruction!(SystemCall, RType),
    /* 0x0D */ instruction!(Break, RType),
    /* 0x0E */ instruction!(Invalid, Invalid),
    /* 0x0F */ instruction!(Invalid, Invalid),
    /* 0x10 */ instruction!(MoveFromHi, RType),
    /* 0x11 */ instruction!(MoveToHi, RType),
    /* 0x12 */ instruction!(MoveFromLo, RType),
    /* 0x13 */ instruction!(MoveToLo, RType),
    /* 0x14 */ instruction!(Invalid, Invalid),
    /* 0x15 */ instruction!(Invalid, Invalid),
    /* 0x16 */ instruction!(Invalid, Invalid),
    /* 0x17 */ instruction!(Invalid, Invalid),
    /* 0x18 */ instruction!(Multiply, RType),
    /* 0x19 */ instruction!(MultiplyUnsigned, RType),
    /* 0x1A */ instruction!(Divide, RType),
    /* 0x1B */ instruction!(DivideUnsigned, RType),
    /* 0x1C */ instruction!(Invalid, Invalid),
    /* 0x1D */ instruction!(Invalid, Invalid),
    /* 0x1E */ instruction!(Invalid, Invalid),
    /* 0x1F */ instruction!(Invalid, Invalid),
    /* 0x20 */ instruction!(Add, RType),
    /* 0x21 */ instruction!(AddUnsigned, RType),
    /* 0x22 */ instruction!(Sub, RType),
    /* 0x23 */ instruction!(SubUnsigned, RType),
    /* 0x24 */ instruction!(And, RType),
    /* 0x25 */ instruction!(Or, RType),
    /* 0x26 */ instruction!(Xor, RType),
    /* 0x27 */ instruction!(Nor, RType),
    /* 0x28 */ instruction!(Invalid, Invalid),
    /* 0x29 */ instruction!(Invalid, Invalid),
    /* 0x2A */ instruction!(SetLessThan, RType),
    /* 0x2B */ instruction!(SetLessThanUnsigned, RType),
    /* 0x2C */ instruction!(Invalid, Invalid),
    /* 0x2D */ instruction!(Invalid, Invalid),
    /* 0x2E */ instruction!(Invalid, Invalid),
    /* 0x2F */ instruction!(Invalid, Invalid),
    /* 0x30 */ instruction!(Invalid, Invalid),
    /* 0x31 */ instruction!(Invalid, Invalid),
    /* 0x32 */ instruction!(Invalid, Invalid),
    /* 0x33 */ instruction!(Invalid, Invalid),
    /* 0x34 */ instruction!(Invalid, Invalid),
    /* 0x35 */ instruction!(Invalid, Invalid),
    /* 0x36 */ instruction!(Invalid, Invalid),
    /* 0x37 */ instruction!(Invalid, Invalid),
    /* 0x38 */ instruction!(Invalid, Invalid),
    /* 0x39 */ instruction!(Invalid, Invalid),
    /* 0x3A */ instruction!(Invalid, Invalid),
    /* 0x3B */ instruction!(Invalid, Invalid),
    /* 0x3C */ instruction!(Invalid, Invalid),
    /* 0x3D */ instruction!(Invalid, Invalid),
    /* 0x3E */ instruction!(Invalid, Invalid),
    /* 0x3F */ instruction!(Invalid, Invalid),
];

// opcode = 0x01 - REGIMM instructions (uses rt field bits 20-16, so 32 possible values)
static MIPS_REGIMM_LUT: [Instruction; 32] = [
    /* 0x00 */ instruction!(BranchLessThanZero, IType),
    /* 0x01 */ instruction!(BranchGreaterEqualZero, IType),
    /* 0x02 */ instruction!(Invalid, Invalid),
    /* 0x03 */ instruction!(Invalid, Invalid),
    /* 0x04 */ instruction!(Invalid, Invalid),
    /* 0x05 */ instruction!(Invalid, Invalid),
    /* 0x06 */ instruction!(Invalid, Invalid),
    /* 0x07 */ instruction!(Invalid, Invalid),
    /* 0x08 */ instruction!(Invalid, Invalid),
    /* 0x09 */ instruction!(Invalid, Invalid),
    /* 0x0A */ instruction!(Invalid, Invalid),
    /* 0x0B */ instruction!(Invalid, Invalid),
    /* 0x0C */ instruction!(Invalid, Invalid),
    /* 0x0D */ instruction!(Invalid, Invalid),
    /* 0x0E */ instruction!(Invalid, Invalid),
    /* 0x0F */ instruction!(Invalid, Invalid),
    /* 0x10 */ instruction!(BranchLessThanZeroAndLink, IType),
    /* 0x11 */ instruction!(BranchGreaterEqualZeroAndLink, IType),
    /* 0x12 */ instruction!(Invalid, Invalid),
    /* 0x13 */ instruction!(Invalid, Invalid),
    /* 0x14 */ instruction!(Invalid, Invalid),
    /* 0x15 */ instruction!(Invalid, Invalid),
    /* 0x16 */ instruction!(Invalid, Invalid),
    /* 0x17 */ instruction!(Invalid, Invalid),
    /* 0x18 */ instruction!(Invalid, Invalid),
    /* 0x19 */ instruction!(Invalid, Invalid),
    /* 0x1A */ instruction!(Invalid, Invalid),
    /* 0x1B */ instruction!(Invalid, Invalid),
    /* 0x1C */ instruction!(Invalid, Invalid),
    /* 0x1D */ instruction!(Invalid, Invalid),
    /* 0x1E */ instruction!(Invalid, Invalid),
    /* 0x1F */ instruction!(Invalid, Invalid),
];

// opcode = anything else
static MIPS_OTHER_LUT: [Instruction; 64] = [
    /* 0x00 */ instruction!(Invalid, Invalid), // SPECIAL
    /* 0x01 */ instruction!(Invalid, Invalid), // REGIMM
    /* 0x02 */ instruction!(Jump, JType),
    /* 0x03 */ instruction!(JumpAndLink, JType),
    /* 0x04 */ instruction!(BranchEqual, IType),
    /* 0x05 */ instruction!(BranchNotEqual, IType),
    /* 0x06 */ instruction!(BranchLessEqualZero, IType),
    /* 0x07 */ instruction!(BranchGreaterThanZero, IType),
    /* 0x08 */ instruction!(AddImmediate, IType),
    /* 0x09 */ instruction!(AddImmediateUnsigned, IType),
    /* 0x0A */ instruction!(SetLessThanImmediate, IType),
    /* 0x0B */ instruction!(SetLessThanImmediateUnsigned, IType),
    /* 0x0C */ instruction!(AndImmediate, IType),
    /* 0x0D */ instruction!(OrImmediate, IType),
    /* 0x0E */ instruction!(XorImmediate, IType),
    /* 0x0F */ instruction!(LoadUpperImmediate, IType),
    /* 0x10 */ instruction!(Invalid, Invalid),
    /* 0x11 */ instruction!(Invalid, Invalid),
    /* 0x12 */ instruction!(Invalid, Invalid),
    /* 0x13 */ instruction!(Invalid, Invalid),
    /* 0x14 */ instruction!(Invalid, Invalid),
    /* 0x15 */ instruction!(Invalid, Invalid),
    /* 0x16 */ instruction!(Invalid, Invalid),
    /* 0x17 */ instruction!(Invalid, Invalid),
    /* 0x18 */ instruction!(Invalid, Invalid), // COP0
    /* 0x19 */ instruction!(Invalid, Invalid), // No FPU for PSX
    /* 0x1A */ instruction!(Invalid, Invalid),
    /* 0x1B */ instruction!(Invalid, Invalid),
    /* 0x1C */ instruction!(Invalid, Invalid),
    /* 0x1D */ instruction!(Invalid, Invalid),
    /* 0x1E */ instruction!(Invalid, Invalid),
    /* 0x1F */ instruction!(Invalid, Invalid),
    /* 0x20 */ instruction!(LoadByte, IType),
    /* 0x21 */ instruction!(LoadHalfword, IType),
    /* 0x22 */ instruction!(LoadWordLeft, IType),
    /* 0x23 */ instruction!(LoadWord, IType),
    /* 0x24 */ instruction!(LoadByteUnsigned, IType),
    /* 0x25 */ instruction!(LoadHalfwordUnsigned, IType),
    /* 0x26 */ instruction!(LoadWordRight, IType),
    /* 0x27 */ instruction!(Invalid, Invalid),
    /* 0x28 */ instruction!(StoreByte, IType),
    /* 0x29 */ instruction!(StoreHalfword, IType),
    /* 0x2A */ instruction!(StoreWordLeft, IType),
    /* 0x2B */ instruction!(StoreWord, IType),
    /* 0x2C */ instruction!(Invalid, Invalid),
    /* 0x2D */ instruction!(Invalid, Invalid),
    /* 0x2E */ instruction!(StoreWordRight, IType),
    /* 0x2F */ instruction!(Invalid, Invalid),
    /* 0x30 */ instruction!(Invalid, Invalid),
    /* 0x31 */ instruction!(Invalid, Invalid),
    /* 0x32 */ instruction!(Invalid, Invalid),
    /* 0x33 */ instruction!(Invalid, Invalid),
    /* 0x34 */ instruction!(Invalid, Invalid),
    /* 0x35 */ instruction!(Invalid, Invalid),
    /* 0x36 */ instruction!(Invalid, Invalid),
    /* 0x37 */ instruction!(Invalid, Invalid),
    /* 0x38 */ instruction!(Invalid, Invalid),
    /* 0x39 */ instruction!(Invalid, Invalid),
    /* 0x3A */ instruction!(Invalid, Invalid),
    /* 0x3B */ instruction!(Invalid, Invalid),
    /* 0x3C */ instruction!(Invalid, Invalid),
    /* 0x3D */ instruction!(Invalid, Invalid),
    /* 0x3E */ instruction!(Invalid, Invalid),
    /* 0x3F */ instruction!(Invalid, Invalid),
];

// In reality we'll also add CPU handlers to the LUT, the handlers can then have const generics like UNSIGNED and IMMEDIATE
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Opcode {
    // ALU
    Add,
    AddUnsigned,
    AddImmediate,
    AddImmediateUnsigned,
    Sub,
    SubUnsigned,
    Multiply,
    MultiplyUnsigned,
    Divide,
    DivideUnsigned,
    And,
    AndImmediate,
    Or,
    OrImmediate,
    Xor,
    XorImmediate,
    Nor,
    SetLessThan,
    SetLessThanImmediate,
    SetLessThanUnsigned,
    SetLessThanImmediateUnsigned,

    // Shifter
    ShiftLeftLogical,
    ShiftRightLogical,
    ShiftRightArithmetic,
    ShiftLeftLogicalVariable,
    ShiftRightLogicalVariable,
    ShiftRightArithmeticVariable,

    // Memory Access
    LoadByte,
    LoadByteUnsigned,
    LoadHalfword,
    LoadHalfwordUnsigned,
    LoadWord,
    LoadWordLeft,
    LoadWordRight,
    LoadUpperImmediate,
    StoreByte,
    StoreHalfword,
    StoreWord,
    StoreWordLeft,
    StoreWordRight,

    // Branch
    BranchEqual,
    BranchNotEqual,
    BranchGreaterThanZero,
    BranchLessEqualZero,
    BranchGreaterEqualZero,
    BranchLessThanZero,
    BranchLessThanZeroAndLink,
    BranchGreaterEqualZeroAndLink,
    Jump,
    JumpAndLink,
    JumpRegister,
    JumpAndLinkRegister,
    SystemCall,
    Break,
    MoveFromHi,
    MoveToHi,
    MoveFromLo,
    MoveToLo,

    // Coprocessor
    MoveFromCoprocessor(u8),
    MoveToCoprocessor(u8),
    ReturnFromException,
    TlbRead,
    TlbWriteRandom,
    TlbWriteIndex,
    TlbProbe,

    // Other
    Invalid,
}

// we probably dont need this info for execution, but it may be useful for implementing Display
// later as we can maybe make 3 common implementations for RType, IType, and JType?
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum InstructionType {
    RType,
    IType,
    JType,
    Cop,
    Invalid,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Instruction {
    pub opcode: Opcode,
    pub opcode_raw: u32,
    pub opcode_type: InstructionType,
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Register(u8);

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Operand {
    Register(Register),
    Immediate(u32),
    Address(u32),
    MemoryAddress { offset: i16, base: Register },
}

impl std::fmt::Display for Register {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = match self.0 {
            0 => "$zero",
            1 => "$at",
            2 => "$v0",
            3 => "$v1",
            4..=7 => return write!(f, "$a{}", self.0 - 4),
            8..=15 => return write!(f, "$t{}", self.0 - 8),
            16..=23 => return write!(f, "$s{}", self.0 - 16),
            24..=25 => return write!(f, "$t{}", self.0 - 16),
            26..=27 => return write!(f, "$k{}", self.0 - 26),
            28 => "$gp",
            29 => "$sp",
            30 => "$fp",
            31 => "$ra",
            _ => return write!(f, "$invalid"),
        };
        write!(f, "{}", name)
    }
}

impl std::fmt::Display for Operand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Operand::Register(reg) => write!(f, "{}", reg),
            Operand::Immediate(imm) => write!(f, "0x{:X}", imm),
            Operand::Address(addr) => write!(f, "0x{:08X}", addr),
            Operand::MemoryAddress { offset, base } => write!(f, "0x{:X}({})", offset, base),
        }
    }
}

impl Instruction {
    #[inline(always)]
    pub fn decode(opcode: u32) -> Self {
        let op = (opcode >> 26) & 0x3F; // Extract the opcode bits (bits 31-26)

        let instruction = match op {
            0x00 => {
                // R-Type instructions
                let func = opcode & 0x3F; // Extract the function code (bits 5-0)
                MIPS_RTYPE_LUT[func as usize]
            }
            0x01 => {
                // REGIMM instructions
                let rt = (opcode >> 16) & 0x1F; // Extract the rt field (bits 20-16)
                MIPS_REGIMM_LUT[rt as usize]
            }
            0x10..=0x13 => {
                // Coprocessor instructions (COP0, COP1, COP2, COP3)
                let cop_num = (op & 0x3) as u8; // Extract coprocessor number (bits 1-0 of opcode)
                let fmt = (opcode >> 21) & 0x1F; // Extract the format field (bits 25-21)

                match fmt {
                    0x00 => instruction!(MoveFromCoprocessor(cop_num), Cop),
                    0x04 => instruction!(MoveToCoprocessor(cop_num), Cop),
                    0x10 if cop_num == 0 => {
                        // COP0 operations only
                        instruction!(ReturnFromException, Cop)
                    }
                    _ => instruction!(Invalid, Invalid),
                }
            }
            _ => {
                if op < 64 {
                    MIPS_OTHER_LUT[op as usize]
                } else {
                    instruction!(Invalid, Invalid)
                }
            }
        };

        Instruction {
            opcode_raw: opcode,
            ..instruction
        }
    }

    #[inline(always)]
    pub fn op(&self) -> u8 {
        ((self.opcode_raw >> 26) & 0x3F) as u8
    }

    #[inline(always)]
    pub fn rs(&self) -> u8 {
        ((self.opcode_raw >> 21) & 0x1F) as u8
    }

    #[inline(always)]
    pub fn rt(&self) -> u8 {
        ((self.opcode_raw >> 16) & 0x1F) as u8
    }

    #[inline(always)]
    pub fn rd(&self) -> u8 {
        ((self.opcode_raw >> 11) & 0x1F) as u8
    }

    #[inline(always)]
    pub fn shamt(&self) -> u8 {
        ((self.opcode_raw >> 6) & 0x1F) as u8
    }

    #[inline(always)]
    pub fn funct(&self) -> u8 {
        (self.opcode_raw & 0x3F) as u8
    }

    #[inline(always)]
    pub fn immediate(&self) -> u16 {
        (self.opcode_raw & 0xFFFF) as u16
    }

    #[inline(always)]
    pub fn address(&self) -> u32 {
        self.opcode_raw & 0x03FFFFFF
    }

    #[inline(always)]
    pub fn jump_target(&self, pc: u32) -> u32 {
        let addr_field = self.address();
        let pc_plus_4 = pc + 4;
        (pc_plus_4 & 0xF0000000) | (addr_field << 2)
    }

    pub fn operand1(&self) -> Option<Operand> {
        match self.opcode_type {
            InstructionType::RType => match self.opcode {
                Opcode::ShiftLeftLogical
                | Opcode::ShiftRightLogical
                | Opcode::ShiftRightArithmetic => Some(Operand::Register(Register(self.rt()))),
                Opcode::JumpRegister | Opcode::JumpAndLinkRegister => {
                    Some(Operand::Register(Register(self.rs())))
                }
                Opcode::MoveFromHi | Opcode::MoveFromLo => {
                    Some(Operand::Register(Register(self.rd())))
                }
                Opcode::MoveToHi | Opcode::MoveToLo => Some(Operand::Register(Register(self.rs()))),
                Opcode::Multiply
                | Opcode::MultiplyUnsigned
                | Opcode::Divide
                | Opcode::DivideUnsigned => Some(Operand::Register(Register(self.rs()))),
                Opcode::SystemCall | Opcode::Break => None,
                _ => Some(Operand::Register(Register(self.rd()))),
            },
            InstructionType::IType => match self.opcode {
                Opcode::LoadUpperImmediate => Some(Operand::Register(Register(self.rt()))),
                Opcode::BranchGreaterThanZero
                | Opcode::BranchLessEqualZero
                | Opcode::BranchGreaterEqualZero
                | Opcode::BranchLessThanZero
                | Opcode::BranchLessThanZeroAndLink
                | Opcode::BranchGreaterEqualZeroAndLink => {
                    Some(Operand::Register(Register(self.rs())))
                }
                _ => Some(Operand::Register(Register(self.rt()))),
            },
            InstructionType::JType => Some(Operand::Address(self.address() << 2)),
            _ => None,
        }
    }

    pub fn operand2(&self) -> Option<Operand> {
        match self.opcode_type {
            InstructionType::RType => match self.opcode {
                Opcode::ShiftLeftLogical
                | Opcode::ShiftRightLogical
                | Opcode::ShiftRightArithmetic => Some(Operand::Immediate(self.shamt() as u32)),
                Opcode::ShiftLeftLogicalVariable
                | Opcode::ShiftRightLogicalVariable
                | Opcode::ShiftRightArithmeticVariable => {
                    Some(Operand::Register(Register(self.rs())))
                }
                Opcode::JumpRegister
                | Opcode::MoveFromHi
                | Opcode::MoveFromLo
                | Opcode::SystemCall
                | Opcode::Break => None,
                Opcode::JumpAndLinkRegister => Some(Operand::Register(Register(self.rd()))),
                Opcode::MoveToHi | Opcode::MoveToLo => None,
                Opcode::Multiply
                | Opcode::MultiplyUnsigned
                | Opcode::Divide
                | Opcode::DivideUnsigned => Some(Operand::Register(Register(self.rt()))),
                _ => Some(Operand::Register(Register(self.rs()))),
            },
            InstructionType::IType => match self.opcode {
                Opcode::LoadUpperImmediate => Some(Operand::Immediate(self.immediate() as u32)),
                Opcode::BranchGreaterThanZero
                | Opcode::BranchLessEqualZero
                | Opcode::BranchGreaterEqualZero
                | Opcode::BranchLessThanZero
                | Opcode::BranchLessThanZeroAndLink
                | Opcode::BranchGreaterEqualZeroAndLink => {
                    Some(Operand::Immediate((self.immediate() as i16) as u32))
                }
                Opcode::BranchEqual | Opcode::BranchNotEqual => {
                    Some(Operand::Register(Register(self.rt())))
                }
                Opcode::LoadByte
                | Opcode::LoadByteUnsigned
                | Opcode::LoadHalfword
                | Opcode::LoadHalfwordUnsigned
                | Opcode::LoadWord
                | Opcode::LoadWordLeft
                | Opcode::LoadWordRight => Some(Operand::MemoryAddress {
                    offset: self.immediate() as i16,
                    base: Register(self.rs()),
                }),
                Opcode::StoreByte
                | Opcode::StoreHalfword
                | Opcode::StoreWord
                | Opcode::StoreWordLeft
                | Opcode::StoreWordRight => Some(Operand::MemoryAddress {
                    offset: self.immediate() as i16,
                    base: Register(self.rs()),
                }),
                _ => Some(Operand::Register(Register(self.rs()))),
            },
            InstructionType::JType => None,
            _ => None,
        }
    }

    pub fn operand3(&self) -> Option<Operand> {
        match self.opcode_type {
            InstructionType::RType => match self.opcode {
                Opcode::ShiftLeftLogical
                | Opcode::ShiftRightLogical
                | Opcode::ShiftRightArithmetic
                | Opcode::ShiftLeftLogicalVariable
                | Opcode::ShiftRightLogicalVariable
                | Opcode::ShiftRightArithmeticVariable => {
                    Some(Operand::Register(Register(self.rd())))
                }
                Opcode::JumpRegister => None,
                Opcode::JumpAndLinkRegister => None,
                Opcode::MoveFromHi
                | Opcode::MoveFromLo
                | Opcode::MoveToHi
                | Opcode::MoveToLo
                | Opcode::SystemCall
                | Opcode::Break
                | Opcode::Multiply
                | Opcode::MultiplyUnsigned
                | Opcode::Divide
                | Opcode::DivideUnsigned => None,
                _ => Some(Operand::Register(Register(self.rt()))),
            },
            InstructionType::IType => match self.opcode {
                Opcode::LoadUpperImmediate
                | Opcode::BranchGreaterThanZero
                | Opcode::BranchLessEqualZero
                | Opcode::BranchGreaterEqualZero
                | Opcode::BranchLessThanZero
                | Opcode::BranchLessThanZeroAndLink
                | Opcode::BranchGreaterEqualZeroAndLink => None,
                Opcode::BranchEqual | Opcode::BranchNotEqual => {
                    Some(Operand::Immediate((self.immediate() as i16) as u32))
                }
                Opcode::LoadByte
                | Opcode::LoadByteUnsigned
                | Opcode::LoadHalfword
                | Opcode::LoadHalfwordUnsigned
                | Opcode::LoadWord
                | Opcode::LoadWordLeft
                | Opcode::LoadWordRight
                | Opcode::StoreByte
                | Opcode::StoreHalfword
                | Opcode::StoreWord
                | Opcode::StoreWordLeft
                | Opcode::StoreWordRight => None,
                _ => Some(Operand::Immediate((self.immediate() as i16) as u32)),
            },
            InstructionType::JType => None,
            _ => None,
        }
    }
}

fn main() {
    for value in 0x00..=0xFF {
        let opcode = (value as u32) << 26;
        let instr = Instruction::decode(opcode);
        if instr.opcode_type == InstructionType::Invalid {
            continue; // Skip invalid instructions
        }

        println!("{opcode:08X} = {}", instr);
    }

    // println!("\n\n");

    // let bios_buffer = include_bytes!("../SCPH1000.BIN");
    // for (i, chunk) in bios_buffer.chunks(4).enumerate() {
    //     if chunk.len() < 4 {
    //         continue;
    //     }

    //     if i > 10 * 4 {
    //         break;
    //     }

    //     let opcode = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    //     let instr = Instruction::decode(opcode);
    //     println!("{:08X}: {opcode:08X} > {}", i * 4, instr);
    //     //dbg!(&instr);
    // }
}

impl std::fmt::Debug for Instruction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "opcode: {}, op: {}, rs: {}, rt: {}, rd: {}, shamt: {}, funct: {}, immediate: {:04X}, address: {:08X}",
            self.opcode,
            self.op(),
            self.rs(),
            self.rt(),
            self.rd(),
            self.shamt(),
            self.funct(),
            self.immediate(),
            self.address()
        )
    }
}

impl std::fmt::Display for Opcode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let opcode_str = match self {
            // ALU
            Opcode::Add => "add",
            Opcode::AddUnsigned => "addu",
            Opcode::AddImmediate => "addi",
            Opcode::AddImmediateUnsigned => "addiu",
            Opcode::Sub => "sub",
            Opcode::SubUnsigned => "subu",
            Opcode::Multiply => "mult",
            Opcode::MultiplyUnsigned => "multu",
            Opcode::Divide => "div",
            Opcode::DivideUnsigned => "divu",
            Opcode::And => "and",
            Opcode::AndImmediate => "andi",
            Opcode::Or => "or",
            Opcode::OrImmediate => "ori",
            Opcode::Xor => "xor",
            Opcode::XorImmediate => "xori",
            Opcode::Nor => "nor",
            Opcode::SetLessThan => "slt",
            Opcode::SetLessThanImmediate => "slti",
            Opcode::SetLessThanUnsigned => "sltu",
            Opcode::SetLessThanImmediateUnsigned => "sltiu",

            // Shifter
            Opcode::ShiftLeftLogical => "sll",
            Opcode::ShiftRightLogical => "srl",
            Opcode::ShiftRightArithmetic => "sra",
            Opcode::ShiftLeftLogicalVariable => "sllv",
            Opcode::ShiftRightLogicalVariable => "srlv",
            Opcode::ShiftRightArithmeticVariable => "srav",

            // Memory Access
            Opcode::LoadByte => "lb",
            Opcode::LoadByteUnsigned => "lbu",
            Opcode::LoadHalfword => "lh",
            Opcode::LoadHalfwordUnsigned => "lhu",
            Opcode::LoadWord => "lw",
            Opcode::LoadWordLeft => "lwl",
            Opcode::LoadWordRight => "lwr",
            Opcode::LoadUpperImmediate => "lui",
            Opcode::StoreByte => "sb",
            Opcode::StoreHalfword => "sh",
            Opcode::StoreWord => "sw",
            Opcode::StoreWordLeft => "swl",
            Opcode::StoreWordRight => "swr",

            // Branch
            Opcode::BranchEqual => "beq",
            Opcode::BranchNotEqual => "bne",
            Opcode::BranchGreaterThanZero => "bgtz",
            Opcode::BranchLessEqualZero => "blez",
            Opcode::BranchGreaterEqualZero => "bgez",
            Opcode::BranchLessThanZero => "bltz",
            Opcode::BranchLessThanZeroAndLink => "bltzal",
            Opcode::BranchGreaterEqualZeroAndLink => "bgezal",
            Opcode::Jump => "j",
            Opcode::JumpAndLink => "jal",
            Opcode::JumpRegister => "jr",
            Opcode::JumpAndLinkRegister => "jalr",
            Opcode::SystemCall => "syscall",
            Opcode::Break => "break",
            Opcode::MoveFromHi => "mfhi",
            Opcode::MoveToHi => "mthi",
            Opcode::MoveFromLo => "mflo",
            Opcode::MoveToLo => "mtlo",

            // Coprocessor
            Opcode::MoveFromCoprocessor(cop) => return write!(f, "mfc{}", cop),
            Opcode::MoveToCoprocessor(cop) => return write!(f, "mtc{}", cop),
            Opcode::ReturnFromException => "rfe",
            Opcode::TlbRead => "tlbr",
            Opcode::TlbWriteRandom => "tlbwr",
            Opcode::TlbWriteIndex => "tlbwi",
            Opcode::TlbProbe => "tlbp",

            // Other
            Opcode::Invalid => "???",
        };
        write!(f, "{}", opcode_str)
    }
}

impl std::fmt::Display for Instruction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut parts = vec![format!("{}", self.opcode)];

        if let Some(op1) = self.operand1() {
            parts.push(format!("{}", op1));
        }

        if let Some(op2) = self.operand2() {
            parts.push(format!("{}", op2));
        }

        if let Some(op3) = self.operand3() {
            parts.push(format!("{}", op3));
        }

        if parts.len() == 1 {
            write!(f, "{}", parts[0])
        } else {
            write!(f, "{} {}", parts[0], parts[1..].join(", "))
        }
    }
}
