using KDS.e6502.OpCodes;
using KDS.e6502.Utility;

namespace KDS.e6502
{
    public enum e6502Type { CMOS, NMOS }

    public class CPU6502
    {
        // Registers
        public byte A, X, Y, SP;
        public ushort PC;
        private byte P; // Status register (NV-BDIZC)
        public bool NF { get => (P & 0x80) != 0; set => P = (byte)((P & 0x7f) | (value ? 0x80 : 0)); }
        public bool VF { get => (P & 0x40) != 0; set => P = (byte)((P & 0xbf) | (value ? 0x40 : 0)); }
        public bool DF { get => (P & 0x08) != 0; set => P = (byte)((P & 0xf7) | (value ? 0x08 : 0)); }
        public bool IF { get => (P & 0x04) != 0; set => P = (byte)((P & 0xfb) | (value ? 0x04 : 0)); }
        public bool ZF { get => (P & 0x02) != 0; set => P = (byte)((P & 0xfd) | (value ? 0x02 : 0)); }
        public bool CF { get => (P & 0x01) != 0; set => P = (byte)((P & 0xfe) | (value ? 0x01 : 0)); }

        // Interrupt flags
        public bool IRQWaiting, NMIWaiting;
        protected bool RDY;

        private readonly OpCodeTable opCodeTable = new();
        private readonly e6502Type CPUType;
        private bool Prefetched;
        private int PrefetchedOperand;
        public IBusDevice SystemBus { get; }
		private OpCodeRecord currentOp = new();
		
        public CPU6502(IBusDevice bus, e6502Type cpuType = e6502Type.NMOS)
        {
            SystemBus = bus;
            CPUType = cpuType;
            P = 0x04; // IF starts true
        }

        public void Boot(ushort pc = 0)
        {
            PC = pc == 0 ? ReadWord(0xfffc) : pc;
            P = 0x04; // IF = true
            NMIWaiting = IRQWaiting = false;
        }

        public string DasmNextInstruction() =>
            opCodeTable.OpCodes[SystemBus.Read(PC)].Dasm(
                opCodeTable.OpCodes[SystemBus.Read(PC)].Bytes == 3 ? GetImmWord() : GetImmByte());

        public int ClocksForNext()
        {
            int clocks = ProcessInterrupts() ? 7 : 0; // 7 cycles for interrupts
            currentOp = opCodeTable.OpCodes[SystemBus.Read(PC)];
            PrefetchedOperand = GetOperand(currentOp.AddressMode, out bool crossBoundary);
            clocks += currentOp.Cycles + 
                     (CPUType == e6502Type.CMOS ? ClocksForCMOS() : 0) + 
                     ClocksForBranching() + 
                     (crossBoundary && currentOp.CheckPageBoundary ? 1 : 0);
            Prefetched = true;
            return clocks;
        }

        private int ClocksForCMOS() => currentOp.OpCode switch
        {
            0x6c => 1, // JMP indirect fix
            0x7d or 0xfd => DF ? 1 : 0, // ADC/SBC abs,X/Y in decimal mode
            0x1e or 0x3e or 0x5e or 0x7e => -1, // ASL/LSR/ROL/ROR abs,X
            _ => 0
        };

        private int ClocksForBranching() => currentOp.OpCode switch
        {
            0x90 => BranchCycles(!CF), // BCC
            0xb0 => BranchCycles(CF),  // BCS
            0xf0 => BranchCycles(ZF),  // BEQ
            0x30 => BranchCycles(NF),  // BMI
            0xd0 => BranchCycles(!ZF), // BNE
            0x10 => BranchCycles(!NF), // BPL
            0x80 => BranchCycles(true), // BRA
            0x50 => BranchCycles(!VF), // BVC
            0x70 => BranchCycles(VF),  // BVS
            _ => 0
        };

        private int BranchCycles(bool taken) => 
            taken ? (((PC + 2) & 0xff00) != ((PC + 2 + PrefetchedOperand) & 0xff00) ? 2 : 1) : 0;

        public void ExecuteNext()
        {
            if (!Prefetched && ProcessInterrupts()) return;
            ExecuteInstruction(Prefetched ? PrefetchedOperand : 
                (currentOp = opCodeTable.OpCodes[SystemBus.Read(PC)]).AddressMode switch
                {
                    _ => GetOperand(currentOp.AddressMode, out _)
                });
            Prefetched = false;
        }

        private bool ProcessInterrupts()
        {
            if (NMIWaiting) { DoIRQ(0xfffa, false); NMIWaiting = false; return true; }
            if (!IF && IRQWaiting) { DoIRQ(0xfffe, false); IRQWaiting = false; return true; }
            return false;
        }

        private void ExecuteInstruction(int oper)
        {
            int result;
            byte temp;

            switch (currentOp.OpCode)
            {
                // ADC
                case 0x61 or 0x65 or 0x69 or 0x6d or 0x71 or 0x72 or 0x75 or 0x79 or 0x7d:
                    if (DF) DecimalADC(oper); else ADC((byte)oper); break;

                // AND
                case 0x21 or 0x25 or 0x29 or 0x2d or 0x31 or 0x32 or 0x35 or 0x39 or 0x3d:
                    A = (byte)(result = A & oper); SetNZ(result); break;

                // ASL
                case 0x06 or 0x16 or 0x0a or 0x0e or 0x1e:
                    CF = oper >= 0x80; result = oper << 1; SetNZ(result); SaveOperand(result); break;

                // BBR
                case 0x0f or 0x1f or 0x2f or 0x3f or 0x4f or 0x5f or 0x6f or 0x7f:
    			if ((oper & (1 << (currentOp.OpCode >> 4))) == 0) PC = (ushort)(PC + (sbyte)SystemBus.Read((ushort)(PC + 2))); break;
				
                // BBS
                case 0x8f or 0x9f or 0xaf or 0xbf or 0xcf or 0xdf or 0xef or 0xff:
    			if ((oper & (1 << ((currentOp.OpCode & 0x70) >> 4))) != 0) PC = (ushort)(PC + (sbyte)SystemBus.Read((ushort)(PC + 2))); break;
				
                // Branches
                case 0x90: CheckBranch(!CF, oper); break; // BCC
                case 0xb0: CheckBranch(CF, oper); break;  // BCS
                case 0xf0: CheckBranch(ZF, oper); break;  // BEQ
                case 0x30: CheckBranch(NF, oper); break;  // BMI
                case 0xd0: CheckBranch(!ZF, oper); break; // BNE
                case 0x10: CheckBranch(!NF, oper); break; // BPL
                case 0x80: CheckBranch(true, oper); break; // BRA
                case 0x50: CheckBranch(!VF, oper); break; // BVC
                case 0x70: CheckBranch(VF, oper); break;  // BVS

                // BIT
                case 0x24 or 0x2c or 0x34 or 0x3c or 0x89:
                    result = A & oper;
                    if (currentOp.AddressMode != AddressModes.Immediate) { NF = oper >= 0x80; VF = (oper & 0x40) != 0; }
                    ZF = result == 0; break;

                // BRK
                case 0x00: PC += 2; DoIRQ(0xfffe, true); if (CPUType == e6502Type.CMOS) DF = false; break;

                // Clear flags
                case 0x18: CF = false; break; // CLC
                case 0xd8: DF = false; break; // CLD
                case 0x58: IF = false; break; // CLI
                case 0xb8: VF = false; break; // CLV

                // CMP
                case 0xc1 or 0xc5 or 0xc9 or 0xcd or 0xd1 or 0xd2 or 0xd5 or 0xd9 or 0xdd:
                    temp = (byte)(A - oper); SetNZ(temp); CF = A >= (byte)oper; break;

                // CPX
                case 0xe0 or 0xe4 or 0xec:
                    temp = (byte)(X - oper); SetNZ(temp); CF = X >= (byte)oper; break;

                // CPY
                case 0xc0 or 0xc4 or 0xcc:
                    temp = (byte)(Y - oper); SetNZ(temp); CF = Y >= (byte)oper; break;

                // DEC
                case 0xc6 or 0xce or 0xd6 or 0xde or 0x3a:
                    result = oper - 1; SetNZ(result); SaveOperand(result); break;

                // DEX/DEY
                case 0xca: X = (byte)(result = X - 1); SetNZ(result); break;
                case 0x88: Y = (byte)(result = Y - 1); SetNZ(result); break;

                // EOR
                case 0x41 or 0x45 or 0x49 or 0x4d or 0x51 or 0x52 or 0x55 or 0x59 or 0x5d:
                    A = (byte)(result = A ^ oper); SetNZ(result); break;

                // INC
                case 0xe6 or 0xee or 0xf6 or 0xfe or 0x1a:
                    result = oper + 1; SetNZ(result); SaveOperand(result); break;

                // INX/INY
                case 0xe8: X = (byte)(result = X + 1); SetNZ(result); break;
                case 0xc8: Y = (byte)(result = Y + 1); SetNZ(result); break;

                // JMP
                case 0x4c: PC = (ushort)oper; return;
                case 0x6c: PC = ReadWord((ushort)oper); return;
                case 0x7c: PC = ReadWord((ushort)(oper + X)); return;

                // JSR
                case 0x20: PushWord((ushort)(PC + 2)); PC = (ushort)oper; return;

                // LDA/LDX/LDY
                case 0xa1 or 0xa5 or 0xa9 or 0xad or 0xb1 or 0xb2 or 0xb5 or 0xb9 or 0xbd:
                    A = (byte)(result = oper); SetNZ(result); break;
                case 0xa2 or 0xa6 or 0xae or 0xb6 or 0xbe:
                    X = (byte)(result = oper); SetNZ(result); break;
                case 0xa0 or 0xa4 or 0xac or 0xb4 or 0xbc:
                    Y = (byte)(result = oper); SetNZ(result); break;

                // LSR
                case 0x46 or 0x4a or 0x4e or 0x56 or 0x5e:
                    CF = (oper & 1) != 0; result = oper >> 1; SetNZ(result); SaveOperand(result); break;

                // NOP
                case 0xea: break;

                // ORA
                case 0x01 or 0x05 or 0x09 or 0x0d or 0x11 or 0x12 or 0x15 or 0x19 or 0x1d:
                    A = (byte)(result = A | oper); SetNZ(result); break;

                // Push
                case 0x48: PushByte(A); break; // PHA
                case 0x08: PushByte((byte)(P | 0x30)); break; // PHP (B flag set)
                case 0xda: PushByte(X); break; // PHX
                case 0x5a: PushByte(Y); break; // PHY

                // Pull
                case 0x68: A = (byte)(result = PopByte()); SetNZ(result); break; // PLA
                case 0x28: P = (byte)(PopByte() & 0xcf); break; // PLP (clear B flag)
                case 0xfa: X = (byte)(result = PopByte()); SetNZ(result); break; // PLX
                case 0x7a: Y = (byte)(result = PopByte()); SetNZ(result); break; // PLY

                // RMB
                case 0x07 or 0x17 or 0x27 or 0x37 or 0x47 or 0x57 or 0x67 or 0x77:
                    SaveOperand(oper & ~(1 << (currentOp.OpCode >> 4))); break;

                // SMB
                case 0x87 or 0x97 or 0xa7 or 0xb7 or 0xc7 or 0xd7 or 0xe7 or 0xf7:
                    SaveOperand(oper | (1 << ((currentOp.OpCode & 0x70) >> 4))); break;

                // ROL
                case 0x26 or 0x2a or 0x2e or 0x36 or 0x3e:
                    result = (oper << 1) | (CF ? 1 : 0); CF = oper >= 0x80; SetNZ(result); SaveOperand(result); break;

                // ROR
                case 0x66 or 0x6a or 0x6e or 0x76 or 0x7e:
                    result = (oper >> 1) | (CF ? 0x80 : 0); CF = (oper & 1) != 0; SetNZ(result); SaveOperand(result); break;

                // RTI
                case 0x40: P = (byte)(PopByte() & 0xcf); PC = PopWord(); break;

                // RTS
                case 0x60: PC = (ushort)(PopWord() + 1); return;

                // SBC
                case 0xe1 or 0xe5 or 0xe9 or 0xed or 0xf1 or 0xf2 or 0xf5 or 0xf9 or 0xfd:
                    if (DF) DecimalSBC(oper); else ADC((byte)~oper); break;

                // Set flags
                case 0x38: CF = true; break; // SEC
                case 0xf8: DF = true; break; // SED
                case 0x78: IF = true; break; // SEI

                // STA/STX/STY/STZ
                case 0x81 or 0x85 or 0x8d or 0x91 or 0x92 or 0x95 or 0x99 or 0x9d:
                    SaveOperand(A); break;
                case 0x86 or 0x8e or 0x96: SaveOperand(X); break;
                case 0x84 or 0x8c or 0x94: SaveOperand(Y); break;
                case 0x64 or 0x74 or 0x9c or 0x9e: SaveOperand(0); break;

                // Transfers
                case 0xaa: X = (byte)(result = A); SetNZ(result); break; // TAX
                case 0xa8: Y = (byte)(result = A); SetNZ(result); break; // TAY
                case 0xba: X = (byte)(result = SP); SetNZ(result); break; // TSX
                case 0x8a: A = (byte)(result = X); SetNZ(result); break; // TXA
                case 0x9a: SP = X; break; // TXS
                case 0x98: A = (byte)(result = Y); SetNZ(result); break; // TYA

                // TRB/TSB
                case 0x14 or 0x1c: SaveOperand(~A & oper); ZF = (A & oper) == 0; break;
                case 0x04 or 0x0c: SaveOperand(A | oper); ZF = (A & oper) == 0; break;

                // Undocumented NOPs and STP/WAI treated as NOP
                default: break;
            }
            PC += currentOp.Bytes;
        }

        private void DecimalADC(int oper)
        {
            int result = CPUMath.HexToBCD(A) + CPUMath.HexToBCD((byte)oper) + (CF ? 1 : 0);
            CF = result > 99; A = CPUMath.BCDToHex(result % 100); ZF = A == 0; NF = A >= 0x80;
        }

        private void DecimalSBC(int oper)
        {
            int result = CPUMath.HexToBCD(A) - CPUMath.HexToBCD((byte)oper) - (CF ? 0 : 1);
            CF = result >= 0; A = CPUMath.BCDToHex(result < 0 ? result + 100 : result); ZF = A == 0; NF = A >= 0x80;
        }

        private void ADC(byte oper)
        {
            ushort result = (ushort)(A + oper + (CF ? 1 : 0));
            CF = result > 0xff; VF = (~(A ^ oper) & (A ^ result) & 0x80) != 0; A = (byte)result; SetNZ(result);
        }

        private void SetNZ(int value) => P = (byte)((P & 0x7c) | (value & 0x80) | (value == 0 ? 2 : 0));

        private void SaveOperand(int value)
        {
            switch (currentOp.AddressMode)
            {
                case AddressModes.Accumulator: A = (byte)value; break;
                case AddressModes.Absolute: SystemBus.Write(GetImmWord(), (byte)value); break;
                case AddressModes.AbsoluteX: SystemBus.Write((ushort)(GetImmWord() + X), (byte)value); break;
                case AddressModes.AbsoluteY: SystemBus.Write((ushort)(GetImmWord() + Y), (byte)value); break;
                case AddressModes.XIndirect: SystemBus.Write(ReadWord((byte)(GetImmByte() + X)), (byte)value); break;
                case AddressModes.IndirectY: SystemBus.Write((ushort)(ReadWord(GetImmByte()) + Y), (byte)value); break;
                case AddressModes.ZeroPage: SystemBus.Write(GetImmByte(), (byte)value); break;
                case AddressModes.ZeroPageX: SystemBus.Write((byte)(GetImmByte() + X), (byte)value); break;
                case AddressModes.ZeroPageY: SystemBus.Write((byte)(GetImmByte() + Y), (byte)value); break;
                case AddressModes.ZeroPage0: SystemBus.Write(ReadWord(GetImmByte()), (byte)value); break;
                case AddressModes.BranchExt: SystemBus.Write(GetImmByte(), (byte)value); break;
            }
        }

        private void CheckBranch(bool flag, int oper) { if (flag) PC += (ushort)oper; }

        private int GetOperand(AddressModes mode, out bool crossBoundary)
        {
            crossBoundary = false;
            switch (mode)
            {
                case AddressModes.Accumulator: return A;
                case AddressModes.Absolute: return SystemBus.Read(GetImmWord());
                case AddressModes.AbsoluteX: return GetIndexed(GetImmWord(), X, ref crossBoundary);
                case AddressModes.AbsoluteY: return GetIndexed(GetImmWord(), Y, ref crossBoundary);
                case AddressModes.Immediate: return GetImmByte();
                case AddressModes.Indirect: return ReadWord(GetImmWord());
                case AddressModes.XIndirect: return SystemBus.Read(ReadWord((byte)(GetImmByte() + X)));
                case AddressModes.IndirectY: return GetIndexed(ReadWord(GetImmByte()), Y, ref crossBoundary);
                case AddressModes.Relative: return (sbyte)GetImmByte();
                case AddressModes.ZeroPage: return SystemBus.Read(GetImmByte());
                case AddressModes.ZeroPageX: return SystemBus.Read((byte)(GetImmByte() + X));
                case AddressModes.ZeroPageY: return SystemBus.Read((byte)(GetImmByte() + Y));
                case AddressModes.ZeroPage0: return SystemBus.Read(ReadWord(GetImmByte()));
                case AddressModes.BranchExt: return SystemBus.Read(GetImmByte());
                default: return 0;
            }
        }

        private int GetIndexed(ushort baseAddr, byte index, ref bool crossBoundary)
        {
            ushort addr = (ushort)(baseAddr + index);
            if (currentOp.CheckPageBoundary) crossBoundary = (baseAddr & 0xff00) != (addr & 0xff00);
            return SystemBus.Read(addr);
        }

        private ushort GetImmWord() => ReadWord((ushort)(PC + 1));
        private byte GetImmByte() => SystemBus.Read((ushort)(PC + 1));
        private ushort ReadWord(ushort addr) => 
            (ushort)((SystemBus.Read((ushort)(addr + 1)) << 8) | SystemBus.Read(addr));

        private void PushByte(byte data) => SystemBus.Write((ushort)(0x0100 | SP--), data);
        private void PushWord(ushort data)
        {
            SystemBus.Write((ushort)(0x0100 | SP--), (byte)(data >> 8));
            SystemBus.Write((ushort)(0x0100 | SP--), (byte)data);
        }

        private byte PopByte() => SystemBus.Read((ushort)(0x0100 | ++SP));
        private ushort PopWord() => 
            (ushort)((SystemBus.Read((ushort)(0x0100 | (SP += 2))) << 8) | SystemBus.Read((ushort)(0x0100 | (SP - 1))));

        private void DoIRQ(ushort vector, bool isBRK)
        {
            PushWord(PC);
            PushByte((byte)(P | (isBRK ? 0x10 : 0) | 0x20));
            IF = true;
            if (CPUType == e6502Type.CMOS && !isBRK) DF = false;
            PC = ReadWord(vector);
        }
    }
}