namespace KDS.e6502
{
    public interface IBusDevice
    {
        byte Read(ushort address);
        void Write(ushort address, byte data);
        
        // New method to expose memory as a Span for zero-copy access
        Span<byte> GetMemory();
    }
}