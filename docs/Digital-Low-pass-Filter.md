# Digital Low-pass Filter

$Output_i = (1-\beta) Output_{i-1} + \beta Sample_i$

or

$Output_i = Output_{i-1} + \beta (Sample_i - Output_{i-1})$

(The second formula is probably more numerically stable.)

$\beta = 1 - e^{-2 \pi \frac{F_0}{F_S}}$

$F_0 = -2\pi F_S ln(1-\beta)$


$F_0$ : Cutoff frequency (Hz)  
$F_S$ : Sample Frequency (Hz)  
$\beta$ : Filter factor, $0 \lt \beta \le 1$, higher $\beta$ gives higher $F_0$  

## Alternate Form 1

$\alpha = 1 - \beta$

$Output_i = \alpha Output_{i-1} + (1-\alpha) Sample_i$

$\alpha = e^{-2 \pi \frac{F_0}{F_S}}$

$F_0 = -2\pi F_S ln(\alpha)$

$F_0$ : Cutoff frequency (Hz)  
$F_S$ : Sample Frequency (Hz)  
$\alpha$ : Filter factor, $0 \le \alpha \lt 1$, higher $\alpha$ gives lower $F_0$  

## Alternate Form 2

$c = 1 / \beta$

$Output_i = \frac{(c-1) Output_{i-1} + Sample_i}{c}$

$c$ : Filter Coefficient, $c \ge 0$, higher $c$ gives lower $F_0$  
