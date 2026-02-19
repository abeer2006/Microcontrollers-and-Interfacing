import matplotlib.pyplot as plt

# ---- Cleaned & Valid (raw, filtered) Pairs ----
data = [
(1.63,1.61),
(1.63,1.62),
(1.66,1.64),
(1.73,1.69),
(1.82,1.79),
(1.90,1.86),
(1.97,1.93),
(1.99,1.94),
(2.12,2.12),
(2.26,2.21),
(2.28,2.22),
(2.29,2.24),
(2.31,2.26),
(2.49,2.56),
(2.59,2.57),
(2.60,2.59),
(2.64,2.62),
(2.65,2.64),

# Repeated section (since you pasted multiple blocks)
(1.63,1.61),
(1.63,1.62),
(1.66,1.64),
(1.73,1.69),
(1.82,1.79),
(1.90,1.86),
(1.97,1.93),
(1.99,1.94),
(2.12,2.12),
(2.26,2.21),
(2.28,2.22),
(2.29,2.24),
(2.31,2.26),
(2.49,2.56),
(2.59,2.57),
(2.60,2.59),
(2.64,2.62),
(2.65,2.64),

# Final stable block
(1.68,1.70),
(1.68,1.70),
(1.67,1.69),
(1.67,1.69),
(1.67,1.69),
(1.67,1.68),
(1.67,1.68),
(1.66,1.68),
(1.67,1.67),
(1.67,1.67),
(1.67,1.67),
(1.67,1.67),
(1.67,1.67),
(1.68,1.67),
(1.68,1.67),
(1.67,1.67),
(1.68,1.67),
(1.68,1.67),
(1.67,1.67),
(1.68,1.67),
(1.68,1.68),
(1.69,1.68),
(1.69,1.68),
(1.69,1.69),
]

# ---- Separate raw & filtered ----
raw_values = [d[0] for d in data]
filtered_values = [d[1] for d in data]
time = list(range(len(data)))

# ---- Plot ----
plt.figure()
plt.plot(time, raw_values, marker='o', linestyle='-', label='Raw')
plt.plot(time, filtered_values, marker='s', linestyle='-', label='Filtered')

plt.title("Raw vs Filtered Values (Extended Dataset)")
plt.xlabel("Sample Index")
plt.ylabel("Value")
plt.grid(True)
plt.legend()
plt.show()
