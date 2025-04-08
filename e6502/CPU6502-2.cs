using System;
using System.Runtime.CompilerServices;
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
        private readonly Action<int>[] opcodeHandlers = new Action<int>[256];
        public IBusDevice SystemBus { get; }
        private OpCodeRecord currentOp = new();

        public CPU6502(IBusDevice bus, e6502Type cpuType = e6502Type.NMOS)
        {
            SystemBus = bus ?? throw new ArgumentNullException(nameof(bus));
            CPUType = cpuType;
            P = 0x04; // IF starts true
            InitializeOpcodeHandlers();
        }

        public void Boot(ushort pc = 0)
        {
            PC = pc == 0 ? ReadWord(0xfffc) : pc;
            P = 0x04; // IF = true
            NMIWaiting = IRQWaiting = false;
            SP = 0xff; // Stack pointer starts at top of page 1
            RDY = true;
        }

        public string DasmNextInstruction()
        {
            Span<byte> memory = SystemBus.GetMemory();
            return opCodeTable.OpCodes[memory[PC]].Dasm(
                opCodeTable.OpCodes[memory[PC]].Bytes == 3 ? GetImmWord() : GetImmByte());
        }

        public int ClocksForNext()
        {
            int clocks = ProcessInterrupts() ? 7 : 0;
            Span<byte> memory = SystemBus.GetMemory();
            currentOp = opCodeTable.OpCodes[memory[PC]];
            PrefetchedOperand = GetOperand(currentOp.AddressMode, out bool crossBoundary);
            clocks += currentOp.Cycles +
                     (crossBoundary && currentOp.CheckPageBoundary ? 1 : 0) +
                     (CPUType == e6502Type.CMOS ? ClocksForCMOS() : 0);
            Prefetched = true;
            return clocks;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int ClocksForCMOS() => currentOp.OpCode switch
        {
            0x6c => 1, // JMP indirect fix
            0x7d or 0xfd => DF ? 1 : 0, // ADC/SBC abs,X/Y in decimal mode
            0x1e or 0x3e or 0x5e or 0x7e => -1, // ASL/LSR/ROL/ROR abs,X
            _ => 0
        };

        public void ExecuteNext()
        {
            if (!RDY) return;
            if (!Prefetched && ProcessInterrupts()) return;
            Span<byte> memory = SystemBus.GetMemory();
            if (!Prefetched)
            {
                currentOp = opCodeTable.OpCodes[memory[PC]];
                PrefetchedOperand = GetOperand(currentOp.AddressMode, out _);
            }
            opcodeHandlers[currentOp.OpCode](PrefetchedOperand);
            Prefetched = false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool ProcessInterrupts()
        {
            if (NMIWaiting) { DoIRQ(0xfffa, false); NMIWaiting = false; return true; }
            if (!IF && IRQWaiting) { DoIRQ(0xfffe, false); IRQWaiting = false; return true; }
            return false;
        }

        private void InitializeOpcodeHandlers()
        {
            // ADC
            opcodeHandlers[0x61] = oper => { if (DF) DecimalADC(oper); else ADC((byte)oper); PC += 2; }; // (zp,X)
            opcodeHandlers[0x65] = oper => { if (DF) DecimalADC(oper); else ADC((byte)oper); PC += 2; }; // zp
            opcodeHandlers[0x69] = oper => { if (DF) DecimalADC(oper); else ADC((byte)oper); PC += 2; }; // #imm
            opcodeHandlers[0x6d] = oper => { if (DF) DecimalADC(oper); else ADC((byte)oper); PC += 3; }; // abs
            opcodeHandlers[0x71] = oper => { if (DF) DecimalADC(oper); else ADC((byte)oper); PC += 2; }; // (zp),Y
            opcodeHandlers[0x75] = oper => { if (DF) DecimalADC(oper); else ADC((byte)oper); PC += 2; }; // zp,X
            opcodeHandlers[0x79] = oper => { if (DF) DecimalADC(oper); else ADC((byte)oper); PC += 3; }; // abs,Y
            opcodeHandlers[0x7d] = oper => { if (DF) DecimalADC(oper); else ADC((byte)oper); PC += 3; }; // abs,X

            // AND
            opcodeHandlers[0x21] = oper => { A &= (byte)oper; SetNZ(A); PC += 2; }; // (zp,X)
            opcodeHandlers[0x25] = oper => { A &= (byte)oper; SetNZ(A); PC += 2; }; // zp
            opcodeHandlers[0x29] = oper => { A &= (byte)oper; SetNZ(A); PC += 2; }; // #imm
            opcodeHandlers[0x2d] = oper => { A &= (byte)oper; SetNZ(A); PC += 3; }; // abs
            opcodeHandlers[0x31] = oper => { A &= (byte)oper; SetNZ(A); PC += 2; }; // (zp),Y
            opcodeHandlers[0x35] = oper => { A &= (byte)oper; SetNZ(A); PC += 2; }; // zp,X
            opcodeHandlers[0x39] = oper => { A &= (byte)oper; SetNZ(A); PC += 3; }; // abs,Y
            opcodeHandlers[0x3d] = oper => { A &= (byte)oper; SetNZ(A); PC += 3; }; // abs,X

            // ASL
            opcodeHandlers[0x06] = oper => { int r = oper << 1; CF = oper >= 0x80; SetNZ(r); SaveOperand(r); PC += 2; }; // zp
            opcodeHandlers[0x0a] = _ => { CF = A >= 0x80; A <<= 1; SetNZ(A); PC += 1; }; // A
            opcodeHandlers[0x0e] = oper => { int r = oper << 1; CF = oper >= 0x80; SetNZ(r); SaveOperand(r); PC += 3; }; // abs
            opcodeHandlers[0x16] = oper => { int r = oper << 1; CF = oper >= 0x80; SetNZ(r); SaveOperand(r); PC += 2; }; // zp,X
            opcodeHandlers[0x1e] = oper => { int r = oper << 1; CF = oper >= 0x80; SetNZ(r); SaveOperand(r); PC += 3; }; // abs,X

            // BBR (CMOS only)
            if (CPUType == e6502Type.CMOS)
            {
                opcodeHandlers[0x0f] = oper => { if ((oper & 0x01) == 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0x1f] = oper => { if ((oper & 0x02) == 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0x2f] = oper => { if ((oper & 0x04) == 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0x3f] = oper => { if ((oper & 0x08) == 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0x4f] = oper => { if ((oper & 0x10) == 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0x5f] = oper => { if ((oper & 0x20) == 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0x6f] = oper => { if ((oper & 0x40) == 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0x7f] = oper => { if ((oper & 0x80) == 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
            }

            // BBS (CMOS only)
            if (CPUType == e6502Type.CMOS)
            {
                opcodeHandlers[0x8f] = oper => { if ((oper & 0x01) != 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0x9f] = oper => { if ((oper & 0x02) != 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0xaf] = oper => { if ((oper & 0x04) != 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0xbf] = oper => { if ((oper & 0x08) != 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0xcf] = oper => { if ((oper & 0x10) != 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0xdf] = oper => { if ((oper & 0x20) != 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0xef] = oper => { if ((oper & 0x40) != 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
                opcodeHandlers[0xff] = oper => { if ((oper & 0x80) != 0) PC = (ushort)(PC + (sbyte)SystemBus.GetMemory()[PC + 2] + 3); else PC += 3; };
            }

            // BCC, BCS, BEQ, BMI, BNE, BPL, BRA, BVC, BVS
            opcodeHandlers[0x90] = oper => { if (!CF) PC = (ushort)(PC + (sbyte)oper + 2); else PC += 2; };
            opcodeHandlers[0xb0] = oper => { if (CF) PC = (ushort)(PC + (sbyte)oper + 2); else PC += 2; };
            opcodeHandlers[0xf0] = oper => { if (ZF) PC = (ushort)(PC + (sbyte)oper + 2); else PC += 2; };
            opcodeHandlers[0x30] = oper => { if (NF) PC = (ushort)(PC + (sbyte)oper + 2); else PC += 2; };
            opcodeHandlers[0xd0] = oper => { if (!ZF) PC = (ushort)(PC + (sbyte)oper + 2); else PC += 2; };
            opcodeHandlers[0x10] = oper => { if (!NF) PC = (ushort)(PC + (sbyte)oper + 2); else PC += 2; };
            opcodeHandlers[0x80] = oper => { PC = (ushort)(PC + (sbyte)oper + 2); }; // BRA (CMOS only)
            opcodeHandlers[0x50] = oper => { if (!VF) PC = (ushort)(PC + (sbyte)oper + 2); else PC += 2; };
            opcodeHandlers[0x70] = oper => { if (VF) PC = (ushort)(PC + (sbyte)oper + 2); else PC += 2; };

            // BIT
            opcodeHandlers[0x24] = oper => { ZF = (A & oper) == 0; NF = oper >= 0x80; VF = (oper & 0x40) != 0; PC += 2; }; // zp
            opcodeHandlers[0x2c] = oper => { ZF = (A & oper) == 0; NF = oper >= 0x80; VF = (oper & 0x40) != 0; PC += 3; }; // abs
            opcodeHandlers[0x89] = oper => { ZF = (A & oper) == 0; PC += 2; }; // #imm (CMOS only)

            // BRK
            opcodeHandlers[0x00] = _ => { PC += 2; DoIRQ(0xfffe, true); if (CPUType == e6502Type.CMOS) DF = false; };

            // CLC, CLD, CLI, CLV
            opcodeHandlers[0x18] = _ => { CF = false; PC += 1; };
            opcodeHandlers[0xd8] = _ => { DF = false; PC += 1; };
            opcodeHandlers[0x58] = _ => { IF = false; PC += 1; };
            opcodeHandlers[0xb8] = _ => { VF = false; PC += 1; };

            // CMP
            opcodeHandlers[0xc1] = oper => { byte t = (byte)(A - oper); SetNZ(t); CF = A >= (byte)oper; PC += 2; }; // (zp,X)
            opcodeHandlers[0xc5] = oper => { byte t = (byte)(A - oper); SetNZ(t); CF = A >= (byte)oper; PC += 2; }; // zp
            opcodeHandlers[0xc9] = oper => { byte t = (byte)(A - oper); SetNZ(t); CF = A >= (byte)oper; PC += 2; }; // #imm
            opcodeHandlers[0xcd] = oper => { byte t = (byte)(A - oper); SetNZ(t); CF = A >= (byte)oper; PC += 3; }; // abs
            opcodeHandlers[0xd1] = oper => { byte t = (byte)(A - oper); SetNZ(t); CF = A >= (byte)oper; PC += 2; }; // (zp),Y
            opcodeHandlers[0xd5] = oper => { byte t = (byte)(A - oper); SetNZ(t); CF = A >= (byte)oper; PC += 2; }; // zp,X
            opcodeHandlers[0xd9] = oper => { byte t = (byte)(A - oper); SetNZ(t); CF = A >= (byte)oper; PC += 3; }; // abs,Y
            opcodeHandlers[0xdd] = oper => { byte t = (byte)(A - oper); SetNZ(t); CF = A >= (byte)oper; PC += 3; }; // abs,X

            // CPX
            opcodeHandlers[0xe0] = oper => { byte t = (byte)(X - oper); SetNZ(t); CF = X >= (byte)oper; PC += 2; }; // #imm
            opcodeHandlers[0xe4] = oper => { byte t = (byte)(X - oper); SetNZ(t); CF = X >= (byte)oper; PC += 2; }; // zp
            opcodeHandlers[0xec] = oper => { byte t = (byte)(X - oper); SetNZ(t); CF = X >= (byte)oper; PC += 3; }; // abs

            // CPY
            opcodeHandlers[0xc0] = oper => { byte t = (byte)(Y - oper); SetNZ(t); CF = Y >= (byte)oper; PC += 2; }; // #imm
            opcodeHandlers[0xc4] = oper => { byte t = (byte)(Y - oper); SetNZ(t); CF = Y >= (byte)oper; PC += 2; }; // zp
            opcodeHandlers[0xcc] = oper => { byte t = (byte)(Y - oper); SetNZ(t); CF = Y >= (byte)oper; PC += 3; }; // abs

            // DEC
            opcodeHandlers[0xc6] = oper => { int r = oper - 1; SetNZ(r); SaveOperand(r); PC += 2; }; // zp
            opcodeHandlers[0xce] = oper => { int r = oper - 1; SetNZ(r); SaveOperand(r); PC += 3; }; // abs
            opcodeHandlers[0xd6] = oper => { int r = oper - 1; SetNZ(r); SaveOperand(r); PC += 2; }; // zp,X
            opcodeHandlers[0xde] = oper => { int r = oper - 1; SetNZ(r); SaveOperand(r); PC += 3; }; // abs,X
            opcodeHandlers[0x3a] = _ => { A--; SetNZ(A); PC += 1; }; // A (CMOS only)

            // DEX, DEY
            opcodeHandlers[0xca] = _ => { X--; SetNZ(X); PC += 1; };
            opcodeHandlers[0x88] = _ => { Y--; SetNZ(Y); PC += 1; };

            // EOR
            opcodeHandlers[0x41] = oper => { A ^= (byte)oper; SetNZ(A); PC += 2; }; // (zp,X)
            opcodeHandlers[0x45] = oper => { A ^= (byte)oper; SetNZ(A); PC += 2; }; // zp
            opcodeHandlers[0x49] = oper => { A ^= (byte)oper; SetNZ(A); PC += 2; }; // #imm
            opcodeHandlers[0x4d] = oper => { A ^= (byte)oper; SetNZ(A); PC += 3; }; // abs
            opcodeHandlers[0x51] = oper => { A ^= (byte)oper; SetNZ(A); PC += 2; }; // (zp),Y
            opcodeHandlers[0x55] = oper => { A ^= (byte)oper; SetNZ(A); PC += 2; }; // zp,X
            opcodeHandlers[0x59] = oper => { A ^= (byte)oper; SetNZ(A); PC += 3; }; // abs,Y
            opcodeHandlers[0x5d] = oper => { A ^= (byte)oper; SetNZ(A); PC += 3; }; // abs,X

            // INC
            opcodeHandlers[0xe6] = oper => { int r = oper + 1; SetNZ(r); SaveOperand(r); PC += 2; }; // zp
            opcodeHandlers[0xee] = oper => { int r = oper + 1; SetNZ(r); SaveOperand(r); PC += 3; }; // abs
            opcodeHandlers[0xf6] = oper => { int r = oper + 1; SetNZ(r); SaveOperand(r); PC += 2; }; // zp,X
            opcodeHandlers[0xfe] = oper => { int r = oper + 1; SetNZ(r); SaveOperand(r); PC += 3; }; // abs,X
            opcodeHandlers[0x1a] = _ => { A++; SetNZ(A); PC += 1; }; // A (CMOS only)

            // INX, INY
            opcodeHandlers[0xe8] = _ => { X++; SetNZ(X); PC += 1; };
            opcodeHandlers[0xc8] = _ => { Y++; SetNZ(Y); PC += 1; };

            // JMP
            opcodeHandlers[0x4c] = oper => { PC = (ushort)oper; };
            opcodeHandlers[0x6c] = oper => { PC = ReadWord((ushort)oper); };
            opcodeHandlers[0x7c] = oper => { PC = ReadWord((ushort)(oper + X)); }; // abs,X (CMOS only)

            // JSR
            opcodeHandlers[0x20] = oper => { PushWord((ushort)(PC + 2)); PC = (ushort)oper; };

            // LDA
            opcodeHandlers[0xa1] = oper => { A = (byte)oper; SetNZ(A); PC += 2; }; // (zp,X)
            opcodeHandlers[0xa5] = oper => { A = (byte)oper; SetNZ(A); PC += 2; }; // zp
            opcodeHandlers[0xa9] = oper => { A = (byte)oper; SetNZ(A); PC += 2; }; // #imm
            opcodeHandlers[0xad] = oper => { A = (byte)oper; SetNZ(A); PC += 3; }; // abs
            opcodeHandlers[0xb1] = oper => { A = (byte)oper; SetNZ(A); PC += 2; }; // (zp),Y
            opcodeHandlers[0xb5] = oper => { A = (byte)oper; SetNZ(A); PC += 2; }; // zp,X
            opcodeHandlers[0xb9] = oper => { A = (byte)oper; SetNZ(A); PC += 3; }; // abs,Y
            opcodeHandlers[0xbd] = oper => { A = (byte)oper; SetNZ(A); PC += 3; }; // abs,X

            // LDX
            opcodeHandlers[0xa2] = oper => { X = (byte)oper; SetNZ(X); PC += 2; }; // #imm
            opcodeHandlers[0xa6] = oper => { X = (byte)oper; SetNZ(X); PC += 2; }; // zp
            opcodeHandlers[0xae] = oper => { X = (byte)oper; SetNZ(X); PC += 3; }; // abs
            opcodeHandlers[0xb6] = oper => { X = (byte)oper; SetNZ(X); PC += 2; }; // zp,Y
            opcodeHandlers[0xbe] = oper => { X = (byte)oper; SetNZ(X); PC += 3; }; // abs,Y

            // LDY
            opcodeHandlers[0xa0] = oper => { Y = (byte)oper; SetNZ(Y); PC += 2; }; // #imm
            opcodeHandlers[0xa4] = oper => { Y = (byte)oper; SetNZ(Y); PC += 2; }; // zp
            opcodeHandlers[0xac] = oper => { Y = (byte)oper; SetNZ(Y); PC += 3; }; // abs
            opcodeHandlers[0xb4] = oper => { Y = (byte)oper; SetNZ(Y); PC += 2; }; // zp,X
            opcodeHandlers[0xbc] = oper => { Y = (byte)oper; SetNZ(Y); PC += 3; }; // abs,X

            // LSR
            opcodeHandlers[0x46] = oper => { CF = (oper & 1) != 0; int r = oper >> 1; SetNZ(r); SaveOperand(r); PC += 2; }; // zp
            opcodeHandlers[0x4a] = _ => { CF = (A & 1) != 0; A >>= 1; SetNZ(A); PC += 1; }; // A
            opcodeHandlers[0x4e] = oper => { CF = (oper & 1) != 0; int r = oper >> 1; SetNZ(r); SaveOperand(r); PC += 3; }; // abs
            opcodeHandlers[0x56] = oper => { CF = (oper & 1) != 0; int r = oper >> 1; SetNZ(r); SaveOperand(r); PC += 2; }; // zp,X
            opcodeHandlers[0x5e] = oper => { CF = (oper & 1) != 0; int r = oper >> 1; SetNZ(r); SaveOperand(r); PC += 3; }; // abs,X

            // NOP
            opcodeHandlers[0xea] = _ => { PC += 1; };

            // ORA
            opcodeHandlers[0x01] = oper => { A |= (byte)oper; SetNZ(A); PC += 2; }; // (zp,X)
            opcodeHandlers[0x05] = oper => { A |= (byte)oper; SetNZ(A); PC += 2; }; // zp
            opcodeHandlers[0x09] = oper => { A |= (byte)oper; SetNZ(A); PC += 2; }; // #imm
            opcodeHandlers[0x0d] = oper => { A |= (byte)oper; SetNZ(A); PC += 3; }; // abs
            opcodeHandlers[0x11] = oper => { A |= (byte)oper; SetNZ(A); PC += 2; }; // (zp),Y
            opcodeHandlers[0x15] = oper => { A |= (byte)oper; SetNZ(A); PC += 2; }; // zp,X
            opcodeHandlers[0x19] = oper => { A |= (byte)oper; SetNZ(A); PC += 3; }; // abs,Y
            opcodeHandlers[0x1d] = oper => { A |= (byte)oper; SetNZ(A); PC += 3; }; // abs,X

            // PHA, PHP, PHX, PHY
            opcodeHandlers[0x48] = _ => { PushByte(A); PC += 1; };
            opcodeHandlers[0x08] = _ => { PushByte((byte)(P | 0x30)); PC += 1; }; // B flag set
            opcodeHandlers[0xda] = _ => { PushByte(X); PC += 1; }; // CMOS only
            opcodeHandlers[0x5a] = _ => { PushByte(Y); PC += 1; }; // CMOS only

            // PLA, PLP, PLX, PLY
            opcodeHandlers[0x68] = _ => { A = PopByte(); SetNZ(A); PC += 1; };
            opcodeHandlers[0x28] = _ => { P = (byte)(PopByte() & 0xcf); PC += 1; }; // Clear B flag
            opcodeHandlers[0xfa] = _ => { X = PopByte(); SetNZ(X); PC += 1; }; // CMOS only
            opcodeHandlers[0x7a] = _ => { Y = PopByte(); SetNZ(Y); PC += 1; }; // CMOS only

            // RMB (CMOS only)
            if (CPUType == e6502Type.CMOS)
            {
                opcodeHandlers[0x07] = oper => { SaveOperand(oper & ~0x01); PC += 2; };
                opcodeHandlers[0x17] = oper => { SaveOperand(oper & ~0x02); PC += 2; };
                opcodeHandlers[0x27] = oper => { SaveOperand(oper & ~0x04); PC += 2; };
                opcodeHandlers[0x37] = oper => { SaveOperand(oper & ~0x08); PC += 2; };
                opcodeHandlers[0x47] = oper => { SaveOperand(oper & ~0x10); PC += 2; };
                opcodeHandlers[0x57] = oper => { SaveOperand(oper & ~0x20); PC += 2; };
                opcodeHandlers[0x67] = oper => { SaveOperand(oper & ~0x40); PC += 2; };
                opcodeHandlers[0x77] = oper => { SaveOperand(oper & ~0x80); PC += 2; };
            }

            // SMB (CMOS only)
            if (CPUType == e6502Type.CMOS)
            {
                opcodeHandlers[0x87] = oper => { SaveOperand(oper | 0x01); PC += 2; };
                opcodeHandlers[0x97] = oper => { SaveOperand(oper | 0x02); PC += 2; };
                opcodeHandlers[0xa7] = oper => { SaveOperand(oper | 0x04); PC += 2; };
                opcodeHandlers[0xb7] = oper => { SaveOperand(oper | 0x08); PC += 2; };
                opcodeHandlers[0xc7] = oper => { SaveOperand(oper | 0x10); PC += 2; };
                opcodeHandlers[0xd7] = oper => { SaveOperand(oper | 0x20); PC += 2; };
                opcodeHandlers[0xe7] = oper => { SaveOperand(oper | 0x40); PC += 2; };
                opcodeHandlers[0xf7] = oper => { SaveOperand(oper | 0x80); PC += 2; };
            }

            // ROL
            opcodeHandlers[0x26] = oper => { int r = (oper << 1) | (CF ? 1 : 0); CF = oper >= 0x80; SetNZ(r); SaveOperand(r); PC += 2; }; // zp
            opcodeHandlers[0x2a] = _ => { int r = (A << 1) | (CF ? 1 : 0); CF = A >= 0x80; A = (byte)r; SetNZ(A); PC += 1; }; // A
            opcodeHandlers[0x2e] = oper => { int r = (oper << 1) | (CF ? 1 : 0); CF = oper >= 0x80; SetNZ(r); SaveOperand(r); PC += 3; }; // abs
            opcodeHandlers[0x36] = oper => { int r = (oper << 1) | (CF ? 1 : 0); CF = oper >= 0x80; SetNZ(r); SaveOperand(r); PC += 2; }; // zp,X
            opcodeHandlers[0x3e] = oper => { int r = (oper << 1) | (CF ? 1 : 0); CF = oper >= 0x80; SetNZ(r); SaveOperand(r); PC += 3; }; // abs,X

            // ROR
            opcodeHandlers[0x66] = oper => { int r = (oper >> 1) | (CF ? 0x80 : 0); CF = (oper & 1) != 0; SetNZ(r); SaveOperand(r); PC += 2; }; // zp
            opcodeHandlers[0x6a] = _ => { int r = (A >> 1) | (CF ? 0x80 : 0); CF = (A & 1) != 0; A = (byte)r; SetNZ(A); PC += 1; }; // A
            opcodeHandlers[0x6e] = oper => { int r = (oper >> 1) | (CF ? 0x80 : 0); CF = (oper & 1) != 0; SetNZ(r); SaveOperand(r); PC += 3; }; // abs
            opcodeHandlers[0x76] = oper => { int r = (oper >> 1) | (CF ? 0x80 : 0); CF = (oper & 1) != 0; SetNZ(r); SaveOperand(r); PC += 2; }; // zp,X
            opcodeHandlers[0x7e] = oper => { int r = (oper >> 1) | (CF ? 0x80 : 0); CF = (oper & 1) != 0; SetNZ(r); SaveOperand(r); PC += 3; }; // abs,X

            // RTI
            opcodeHandlers[0x40] = _ => { P = (byte)(PopByte() & 0xcf); PC = PopWord(); };

            // RTS
            opcodeHandlers[0x60] = _ => { PC = (ushort)(PopWord() + 1); };

            // SBC
            opcodeHandlers[0xe1] = oper => { if (DF) DecimalSBC(oper); else ADC((byte)~oper); PC += 2; }; // (zp,X)
            opcodeHandlers[0xe5] = oper => { if (DF) DecimalSBC(oper); else ADC((byte)~oper); PC += 2; }; // zp
            opcodeHandlers[0xe9] = oper => { if (DF) DecimalSBC(oper); else ADC((byte)~oper); PC += 2; }; // #imm
            opcodeHandlers[0xed] = oper => { if (DF) DecimalSBC(oper); else ADC((byte)~oper); PC += 3; }; // abs
            opcodeHandlers[0xf1] = oper => { if (DF) DecimalSBC(oper); else ADC((byte)~oper); PC += 2; }; // (zp),Y
            opcodeHandlers[0xf5] = oper => { if (DF) DecimalSBC(oper); else ADC((byte)~oper); PC += 2; }; // zp,X
            opcodeHandlers[0xf9] = oper => { if (DF) DecimalSBC(oper); else ADC((byte)~oper); PC += 3; }; // abs,Y
            opcodeHandlers[0xfd] = oper => { if (DF) DecimalSBC(oper); else ADC((byte)~oper); PC += 3; }; // abs,X

            // SEC, SED, SEI
            opcodeHandlers[0x38] = _ => { CF = true; PC += 1; };
            opcodeHandlers[0xf8] = _ => { DF = true; PC += 1; };
            opcodeHandlers[0x78] = _ => { IF = true; PC += 1; };

            // STA
            opcodeHandlers[0x81] = oper => { SaveOperand(A); PC += 2; }; // (zp,X)
            opcodeHandlers[0x85] = oper => { SaveOperand(A); PC += 2; }; // zp
            opcodeHandlers[0x8d] = oper => { SaveOperand(A); PC += 3; }; // abs
            opcodeHandlers[0x91] = oper => { SaveOperand(A); PC += 2; }; // (zp),Y
            opcodeHandlers[0x95] = oper => { SaveOperand(A); PC += 2; }; // zp,X
            opcodeHandlers[0x99] = oper => { SaveOperand(A); PC += 3; }; // abs,Y
            opcodeHandlers[0x9d] = oper => { SaveOperand(A); PC += 3; }; // abs,X

            // STX
            opcodeHandlers[0x86] = oper => { SaveOperand(X); PC += 2; }; // zp
            opcodeHandlers[0x8e] = oper => { SaveOperand(X); PC += 3; }; // abs
            opcodeHandlers[0x96] = oper => { SaveOperand(X); PC += 2; }; // zp,Y

            // STY
            opcodeHandlers[0x84] = oper => { SaveOperand(Y); PC += 2; }; // zp
            opcodeHandlers[0x8c] = oper => { SaveOperand(Y); PC += 3; }; // abs
            opcodeHandlers[0x94] = oper => { SaveOperand(Y); PC += 2; }; // zp,X

            // STZ (CMOS only)
            if (CPUType == e6502Type.CMOS)
            {
                opcodeHandlers[0x64] = oper => { SaveOperand(0); PC += 2; }; // zp
                opcodeHandlers[0x74] = oper => { SaveOperand(0); PC += 2; }; // zp,X
                opcodeHandlers[0x9c] = oper => { SaveOperand(0); PC += 3; }; // abs
                opcodeHandlers[0x9e] = oper => { SaveOperand(0); PC += 3; }; // abs,X
            }

            // TAX, TAY, TSX, TXA, TXS, TYA
            opcodeHandlers[0xaa] = _ => { X = A; SetNZ(X); PC += 1; };
            opcodeHandlers[0xa8] = _ => { Y = A; SetNZ(Y); PC += 1; };
            opcodeHandlers[0xba] = _ => { X = SP; SetNZ(X); PC += 1; };
            opcodeHandlers[0x8a] = _ => { A = X; SetNZ(A); PC += 1; };
            opcodeHandlers[0x9a] = _ => { SP = X; PC += 1; };
            opcodeHandlers[0x98] = _ => { A = Y; SetNZ(A); PC += 1; };

            // TRB, TSB (CMOS only)
            if (CPUType == e6502Type.CMOS)
            {
                opcodeHandlers[0x14] = oper => { SaveOperand(~A & oper); ZF = (A & oper) == 0; PC += 2; }; // zp
                opcodeHandlers[0x1c] = oper => { SaveOperand(~A & oper); ZF = (A & oper) == 0; PC += 3; }; // abs
                opcodeHandlers[0x04] = oper => { SaveOperand(A | oper); ZF = (A & oper) == 0; PC += 2; }; // zp
                opcodeHandlers[0x0c] = oper => { SaveOperand(A | oper); ZF = (A & oper) == 0; PC += 3; }; // abs
            }

            // Undocumented NMOS opcodes
            if (CPUType == e6502Type.NMOS)
            {
                // LAX
                opcodeHandlers[0xa7] = oper => { A = X = (byte)oper; SetNZ(A); PC += 2; }; // zp
                opcodeHandlers[0xaf] = oper => { A = X = (byte)oper; SetNZ(A); PC += 3; }; // abs
                opcodeHandlers[0xb3] = oper => { A = X = (byte)oper; SetNZ(A); PC += 2; }; // (zp),Y
                opcodeHandlers[0xb7] = oper => { A = X = (byte)oper; SetNZ(A); PC += 2; }; // zp,Y
                opcodeHandlers[0xbf] = oper => { A = X = (byte)oper; SetNZ(A); PC += 3; }; // abs,Y

                // SAX
                opcodeHandlers[0x87] = oper => { SaveOperand(A & X); PC += 2; }; // zp
                opcodeHandlers[0x8f] = oper => { SaveOperand(A & X); PC += 3; }; // abs
                opcodeHandlers[0x97] = oper => { SaveOperand(A & X); PC += 2; }; // zp,Y
                opcodeHandlers[0x83] = oper => { SaveOperand(A & X); PC += 2; }; // (zp,X)

                // DCP
                opcodeHandlers[0xc7] = oper => { int r = oper - 1; SaveOperand(r); byte t = (byte)(A - r); SetNZ(t); CF = A >= (byte)r; PC += 2; }; // zp
                opcodeHandlers[0xcf] = oper => { int r = oper - 1; SaveOperand(r); byte t = (byte)(A - r); SetNZ(t); CF = A >= (byte)r; PC += 3; }; // abs
                opcodeHandlers[0xd7] = oper => { int r = oper - 1; SaveOperand(r); byte t = (byte)(A - r); SetNZ(t); CF = A >= (byte)r; PC += 2; }; // zp,X
                opcodeHandlers[0xdf] = oper => { int r = oper - 1; SaveOperand(r); byte t = (byte)(A - r); SetNZ(t); CF = A >= (byte)r; PC += 3; }; // abs,X

                // JAM/HLT
                for (byte op = 0x02; op <= 0xf2; op += 0x10)
                    opcodeHandlers[op] = _ => { RDY = false; PC += 1; };

                // NOPs (various forms)
                opcodeHandlers[0x1c] = _ => PC += 3;
                opcodeHandlers[0x3c] = _ => PC += 3;
                opcodeHandlers[0x5c] = _ => PC += 3;
                opcodeHandlers[0x7c] = _ => PC += 3;
                opcodeHandlers[0xdc] = _ => PC += 3;
                opcodeHandlers[0xfc] = _ => PC += 3;
                opcodeHandlers[0x04] = _ => PC += 2;
                opcodeHandlers[0x44] = _ => PC += 2;
                opcodeHandlers[0x64] = _ => PC += 2;
            }

            // Default to NOP-like behavior for unhandled opcodes
            for (int i = 0; i < 256; i++)
                opcodeHandlers[i] ??= _ => PC += opCodeTable.OpCodes[i].Bytes;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void DecimalADC(int oper)
        {
            int result = CPUMath.HexToBCD(A) + CPUMath.HexToBCD((byte)oper) + (CF ? 1 : 0);
            CF = result > 99; A = CPUMath.BCDToHex(result % 100); ZF = A == 0; NF = A >= 0x80;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void DecimalSBC(int oper)
        {
            int result = CPUMath.HexToBCD(A) - CPUMath.HexToBCD((byte)oper) - (CF ? 0 : 1);
            CF = result >= 0; A = CPUMath.BCDToHex(result < 0 ? result + 100 : result); ZF = A == 0; NF = A >= 0x80;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void ADC(byte oper)
        {
            ushort result = (ushort)(A + oper + (CF ? 1 : 0));
            CF = result > 0xff; VF = (~(A ^ oper) & (A ^ result) & 0x80) != 0; A = (byte)result; SetNZ(A);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SetNZ(int value) => P = (byte)((P & 0x7c) | (value & 0x80) | (value == 0 ? 2 : 0));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SaveOperand(int value)
        {
            Span<byte> memory = SystemBus.GetMemory();
            switch (currentOp.AddressMode)
            {
                case AddressModes.Accumulator: A = (byte)value; break;
                case AddressModes.Absolute: memory[ReadWord((ushort)(PC - currentOp.Bytes + 1))] = (byte)value; break;
                case AddressModes.AbsoluteX: memory[(ushort)(ReadWord((ushort)(PC - currentOp.Bytes + 1)) + X)] = (byte)value; break;
                case AddressModes.AbsoluteY: memory[(ushort)(ReadWord((ushort)(PC - currentOp.Bytes + 1)) + Y)] = (byte)value; break;
                case AddressModes.XIndirect: memory[ReadWord((byte)(memory[PC - currentOp.Bytes + 1] + X))] = (byte)value; break;
                case AddressModes.IndirectY: memory[(ushort)(ReadWord(memory[PC - currentOp.Bytes + 1]) + Y)] = (byte)value; break;
                case AddressModes.ZeroPage: memory[memory[PC - currentOp.Bytes + 1]] = (byte)value; break;
                case AddressModes.ZeroPageX: memory[(byte)(memory[PC - currentOp.Bytes + 1] + X)] = (byte)value; break;
                case AddressModes.ZeroPageY: memory[(byte)(memory[PC - currentOp.Bytes + 1] + Y)] = (byte)value; break;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int GetOperand(AddressModes mode, out bool crossBoundary)
        {
            crossBoundary = false;
            Span<byte> memory = SystemBus.GetMemory();
            switch (mode)
            {
                case AddressModes.Accumulator: return A;
                case AddressModes.Immediate: return memory[PC + 1];
                case AddressModes.ZeroPage: return memory[memory[PC + 1]];
                case AddressModes.ZeroPageX: return memory[(byte)(memory[PC + 1] + X)];
                case AddressModes.ZeroPageY: return memory[(byte)(memory[PC + 1] + Y)];
                case AddressModes.Absolute:
                    ushort absAddr = ReadWord((ushort)(PC + 1));
                    return memory[absAddr];
                case AddressModes.AbsoluteX:
                    ushort absXBase = ReadWord((ushort)(PC + 1));
                    ushort absXAddr = (ushort)(absXBase + X);
                    crossBoundary = (absXBase & 0xff00) != (absXAddr & 0xff00);
                    return memory[absXAddr];
                case AddressModes.AbsoluteY:
                    ushort absYBase = ReadWord((ushort)(PC + 1));
                    ushort absYAddr = (ushort)(absYBase + Y);
                    crossBoundary = (absYBase & 0xff00) != (absYAddr & 0xff00);
                    return memory[absYAddr];
                case AddressModes.XIndirect:
                    byte zpXAddr = (byte)(memory[PC + 1] + X);
                    return memory[ReadWord(zpXAddr)];
                case AddressModes.IndirectY:
                    ushort zpAddr = ReadWord(memory[PC + 1]);
                    ushort indYAddr = (ushort)(zpAddr + Y);
                    crossBoundary = (zpAddr & 0xff00) != (indYAddr & 0xff00);
                    return memory[indYAddr];
                case AddressModes.Relative: return (sbyte)memory[PC + 1];
                default: return 0; // Should not occur with proper OpCodeTable
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private ushort GetImmWord() => ReadWord((ushort)(PC + 1));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private byte GetImmByte() => SystemBus.GetMemory()[PC + 1];

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private ushort ReadWord(ushort addr)
        {
            Span<byte> memory = SystemBus.GetMemory();
            return (ushort)((memory[addr + 1] << 8) | memory[addr]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void PushByte(byte data)
        {
            Span<byte> memory = SystemBus.GetMemory();
            memory[0x0100 | SP--] = data;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void PushWord(ushort data)
        {
            Span<byte> memory = SystemBus.GetMemory();
            memory[0x0100 | SP--] = (byte)(data >> 8);
            memory[0x0100 | SP--] = (byte)data;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private byte PopByte()
        {
            Span<byte> memory = SystemBus.GetMemory();
            return memory[0x0100 | ++SP];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private ushort PopWord()
        {
            Span<byte> memory = SystemBus.GetMemory();
            SP += 2;
            return (ushort)((memory[0x0100 | SP] << 8) | memory[0x0100 | (SP - 1)]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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