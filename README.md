# CORDIC Implementation of Tanh and Sigmoid on FPGA

This project consists of a fully pipelined hardware unit that computes **Tanh(x)** and **Sigmoid(x)** using fixed-point arithmetic.

## Different Stages

**1. Input Tuning**

The external input is initially in Q14, while everything internally runs in Q24 using a 28-bit datapath. For Sigmoid(x), we use:

$$
\mathrm{Sigmoid}(x) = \frac{1 + \mathrm{Tanh}(x/2)}{2}
$$

Hence, the input is divided by two at this stage when Sigmoid is selected.

**2. Hyperbolic CORDIC Rotation**

We start from:

$$
x = 1,\qquad y = 0
$$

Each stage drives the residual angle \(z\) towards zero using:

$$
d = (z \geq 0) ? +1 : -1
$$

$$
x' = x + d\left(y \gg s\right)
$$

$$
y' = y + d\left(x \gg s\right)
$$

$$
z' = z - d \cdot \text{atanh}(2^{-s})
$$


**3. Linear CORDIC Division**

Instead of instantiating a hardware divider, each stage drives \(y\) towards zero and accumulates the quotient one binary bit at a time:

$$
d = (y \geq 0) ? +1 : -1
$$

$$
y' = y - d\left(x \gg (j+1)\right)
$$

$$
q' = q + d \cdot 2^{-(j+1)}
$$

**4. Output Stage**

The output stage picks either Tanh(x) or Sigmoid(x), rounds the Q24 result back down to Q14, and sign-extends it to 32 bits to match the output size.

## Implementation Results

* **Target FPGA:** Nexys4-DDR (Xilinx Artix-7 xc7a100tcsg324-1)
* **Maximum Clock Frequency:** 296.3 MHz
* **Throughput:** 1 result per cycle
* **Latency:** N<sub>ROT</sub> + N<sub>DIV</sub> + 4 = 46 clock cycles

### Resource Utilization

* **LUTs:** 2,333 / 63,400 (3.68%)
* **Slice Registers:** 2,753 / 126,800 (2.17%)
* **DSP Blocks:** 0 / 240 (0%)
