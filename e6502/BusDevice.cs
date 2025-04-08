using System;
using System.Runtime.CompilerServices;

namespace KDS.e6502
{
    /// <summary>
    /// Example Bus Device for loading a program into 64KB of RAM.
    /// </summary>
    public class BusDevice : IBusDevice
    {
        private readonly byte[] ram;

        public BusDevice(byte[] program, ushort loadingAddress)
        {
            ram = new byte[0x10000]; // 64KB of RAM
            Load(program, loadingAddress);
        }

        public BusDevice(byte[] program) : this(program, 0) { }

        public void Load(byte[] program)
        {
            Load(program, 0);
        }

        public void Load(byte[] program, int loadingAddress)
        {
            if (program == null) throw new ArgumentNullException(nameof(program));
            if (loadingAddress + program.Length > ram.Length)
                throw new ArgumentException("Program size exceeds available memory.", nameof(loadingAddress));

            program.CopyTo(ram, loadingAddress);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public virtual byte Read(ushort address)
        {
            return ram[address];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public virtual void Write(ushort address, byte data)
        {
            ram[address] = data;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Span<byte> GetMemory()
        {
            return ram.AsSpan();
        }
    }
}