# e6502

This (is/was) a fully implemented Motorola 6502 emulation written in C#. <BR /><BR />
I took the CPU core and had AI try to optimize it further. I have no idea how to test it yet, but it probably mangled OpCodes.<BR />If it does not run properly, I think there is still some interesting information within the diffs. In particular there are faster memory accesses when using .Net 9.0+<BR/>
It should compile the same. I changed it to dotnet9.0 however in the csproj file.
