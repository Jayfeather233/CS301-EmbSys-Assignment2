# CS301 Embedding System Assignment 2
---

This project is to write a Hamming code enc/dec function on STM32.

Features: basic enc/dec, image display, 1bit correction, 2bit detection, dynamic length.

---

Generate Hamming code's parity bit and check can be done just using xor operations without dividing them into different groups like $P_1\ P_2\ ...$

Let $P = xor(Position*DataBit)$, `position` is where data bit is in encoded Hamming code position.

Since `DataBit` can only be 0 or 1, so

$P_1 * 2^0 = P\&1=xor(Position\&1 * DataBit)$ so it will only count DataBit for $Position=\{1, 3, 5, 7...\}$
$P_2 * 2^1 = P\&2=xor(Position\&2 * DataBit)$ so it will only count DataBit for $Position=\{2, 3, 6, 7...\}$
$P_3 * 2^2 = P\&4=xor(Position\&4 * DataBit)$ so it will only count DataBit for $Position=\{4, 5, 6, 7...\}$
$...$

That means $P = (P_n...P_2P_1)_{binary}$

Also true for checking, $C = xor(Position*DataBit)$, $C = (C_n...C_2C_1)_{binary}$

What I've done for **Part Two**

- Capable of performing two-bit error detection.
- Ability to manually adjust the maximum number of input data (both original data and Hamming code) bits and provide a warning when exceeding the maximum number of bits(1 <= length <= 277).
- Ability to dymanically display long hamming code. If cannot display using default 24px font, it will use 2 lines of 12px font. If also exceed that length, it will start to use HEX. In maximum 2 lines of 12px HEX, it can display 288 bits(11 for parity) of hamming code.
- Ability to insert images(for cover).