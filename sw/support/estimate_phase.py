import numpy as np
import scipy.special

def compute_probability(value, sigma):
    """
    A continuous value is measured as either 0 or 1. The measurement has additive noise with standard deviation sigma,
    and the quantization is done by rounding the noisy value to the nearest integer (0 or 1).

    Compute the probability of observing a 1 given the continuous value and the noise standard deviation.
    https://kiedontaa.blogspot.com/2025/02/on-quantization-errors-and-recovering.html

    Args:
        value (float): Value of the continuous variable.
        sigma (float): Standard deviation of the additive noise on top of the continuous value.

    Returns:
        float: Probability of observing a 1.
    """
    p = 0.5 + 0.5 * scipy.special.erf((value - 0.5) / (np.sqrt(2) * sigma))
    return p


tick = 50e-9 # 50 ns
tick_sigma = 4.5e-9 # 4.5 ns

sigma = tick_sigma / tick  # Convert to normalized units

# Generate a range of phase values from 0 to 1 and compute their probabilities
phases = np.linspace(0, 1, 32)
probabilities = []

for phase in phases:
    probabilities.append(compute_probability(phase, sigma))
probabilities = np.array(probabilities)

# Scale the probabilities to the range of 0 to 1
probabilities = (probabilities - np.min(probabilities)) / (np.max(probabilities) - np.min(probabilities))

# Shift the quantization so that the continuous values take on values between -0.5 and 0.5,
# where the probability of observing a 1 is 50% when the continuous value is 0.
phases = phases - 0.5

print(f"Probabilities = {probabilities}")
print(f"Phase values = {phases}")
